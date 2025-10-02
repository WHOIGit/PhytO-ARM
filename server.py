#!/usr/bin/env python3
"""
PhytO-ARM Control Server - Modern web interface for managing PhytO-ARM ROS processes

Enhanced version with:
- Tailwind CSS for modern styling
- HTMX for dynamic frontend interactions
- WebSocket support for real-time log streaming
- Live config editing with validation and ROS parameter updates
"""

import json
import logging
import os

from contextlib import asynccontextmanager
from datetime import datetime
from typing import Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse

from server.models import ProcessInfo, ProcessState
from server.dashboard import PhytoARMServer, _check_ros_connectivity

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)



# Global server instance
server: Optional[PhytoARMServer] = None


@asynccontextmanager
async def lifespan(fastapi_app: FastAPI):
    # Startup
    global server
    config_file_path = os.environ.get('PHYTO_ARM_CONFIG')  # Optional now
    auto_start_processes = os.environ.get('PHYTO_ARM_AUTO_START')  # Optional auto-start
    server = PhytoARMServer(config_file_path, auto_start_processes=auto_start_processes)
    await server.initialize()

    yield

    # Shutdown
    if server:
        await server.shutdown()


# FastAPI app
app = FastAPI(title="PhytO-ARM Control Interface", version="2.0.0", lifespan=lifespan)


@app.get("/", response_class=HTMLResponse)
async def get_dashboard():
    """Main dashboard page with Tailwind CSS and HTMX"""
    try:
        with open("server/templates/dashboard.html", "r", encoding='utf-8') as f:
            content = f.read()
        return HTMLResponse(content=content)
    except FileNotFoundError:
        return HTMLResponse(content="<h1>Dashboard template not found</h1>", status_code=500)


# Static file serving for JS libraries
@app.get("/static/tailwind-3.4.17.js")
async def get_tailwind():
    with open("server/lib/tailwind-3.4.17.js", "r", encoding='utf-8') as f:
        return HTMLResponse(content=f.read(), media_type="application/javascript")


@app.get("/static/htmx-2.0.7.min.js")
async def get_htmx():
    with open("server/lib/htmx-2.0.7.min.js", "r", encoding='utf-8') as f:
        return HTMLResponse(content=f.read(), media_type="application/javascript")


@app.get("/static/htmx-ext-ws-2.0.2.js")
async def get_htmx_ws():
    with open("server/lib/htmx-ext-ws-2.0.2.js", "r", encoding='utf-8') as f:
        return HTMLResponse(content=f.read(), media_type="application/javascript")


# API Endpoints

@app.get("/api/status")
async def api_status():
    """Get status of all processes"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    status = await server.get_status()
    return {
        "processes": {name: info.dict() for name, info in status.items()},
        "config_loaded": server.has_config(),
        "ros_status": {"ready": _check_ros_connectivity()[0], "message": _check_ros_connectivity()[1]}
    }


@app.get("/api/launch_configs")
async def api_launch_configs():
    """Get available launch configurations"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    return server.launch_configs


@app.get("/api/processes/render", response_class=HTMLResponse)
async def api_render_processes():
    """Render processes as HTML for HTMX"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    status = await server.get_status()
    config_loaded = server.has_config()

    # Build HTML for processes
    html_parts = []

    # Add config status banner if no config is loaded
    if not config_loaded:
        html_parts.append(_load_template("config_warning.html"))

    # Add alerts test button
    alerts_disabled = "" if config_loaded else "disabled"
    html_parts.append(f'''
    <div class="mb-6 flex justify-between items-center">
        <h2 class="text-xl font-semibold text-gray-800">Process Control</h2>
        <button
            class="bg-purple-600 hover:bg-purple-700 disabled:bg-gray-400 disabled:cursor-not-allowed text-white px-4 py-2 rounded-lg transition-colors flex items-center"
            hx-post="/api/alerts/test"
            hx-target="#alert-result"
            hx-swap="innerHTML"
            {alerts_disabled}
        >
            <span class="mr-2">🧪</span>
            Test Alerts
        </button>
    </div>
    <div id="alert-result" class="mb-4"></div>
    ''')

    # Get all available processes and their metadata
    all_processes = {}

    # Add core system processes
    for name in ['roscore', 'rosbag']:
        process_info = status.get(name, ProcessInfo(name=name, state=ProcessState.STOPPED))
        metadata = server.get_process_metadata(name)
        all_processes[name] = {
            'info': process_info,
            'metadata': metadata,
            'priority': 1 if name == 'roscore' else 2  # Core processes first
        }

    # Add discovered launch configs
    for name in server.launch_configs:
        process_info = status.get(name, ProcessInfo(name=name, state=ProcessState.STOPPED))
        metadata = server.get_process_metadata(name)
        all_processes[name] = {
            'info': process_info,
            'metadata': metadata,
            'priority': 3  # Launch configs after core
        }

    # Sort processes by priority, then by category, then by name
    def sort_key(item):
        name, data = item
        return (
            data['priority'],
            data['metadata']['category'],
            name
        )

    sorted_processes = sorted(all_processes.items(), key=sort_key)

    # Add processes grid
    html_parts.append('<div class="grid grid-cols-1 md:grid-cols-2 xl:grid-cols-3 gap-6">')

    for name, data in sorted_processes:
        html_parts.append(_render_process_card(data['info'], data['metadata'], config_loaded))

    html_parts.append('</div>')
    return HTMLResponse(content=''.join(html_parts))


def _load_template(template_name: str) -> str:
    """Load HTML template from file"""
    try:
        with open(f"server/templates/{template_name}", "r", encoding='utf-8') as f:
            return f.read()
    except FileNotFoundError:
        logger.error("Template %s not found", template_name)
        return f"<div>Template {template_name} not found</div>"


def _render_process_card(process_info: ProcessInfo, metadata: dict, config_loaded: bool = True) -> str:
    """Render a single process card using template"""
    state = process_info.state.value

    # State-specific styling
    state_colors = {
        'running': 'bg-green-100 text-green-800 border-green-200',
        'stopped': 'bg-gray-100 text-gray-800 border-gray-200',
        'starting': 'bg-yellow-100 text-yellow-800 border-yellow-200',
        'stopping': 'bg-orange-100 text-orange-800 border-orange-200',
        'failed': 'bg-red-100 text-red-800 border-red-200'
    }

    # Category-specific styling for card border
    category_colors = {
        'core': 'border-l-4 border-l-blue-500',
        'logging': 'border-l-4 border-l-purple-500',
        'mission': 'border-l-4 border-l-green-500',
        'simulation': 'border-l-4 border-l-yellow-500',
        'sensors': 'border-l-4 border-l-indigo-500',
        'other': 'border-l-4 border-l-gray-400'
    }

    state_class = state_colors.get(state, 'bg-gray-100 text-gray-800 border-gray-200')
    category_class = category_colors.get(metadata.get('category', 'other'), 'border-l-4 border-l-gray-400')
    is_running = state == 'running'
    is_transitioning = state in ['starting', 'stopping']

    # Only disable buttons during state transitions
    buttons_disabled = is_transitioning
    start_disabled = buttons_disabled or is_running
    stop_disabled = buttons_disabled or not is_running

    # Format timestamps
    started_at = ''
    if process_info.started_at:
        time_str = process_info.started_at.strftime("%Y-%m-%d %H:%M:%S")
        started_at = f'<p class="text-sm text-gray-600 mt-2"><strong>Started:</strong>{time_str}</p>'

    restart_info = ''
    if process_info.restart_count > 0:
        restart_info = f'<p class="text-sm text-gray-600"><strong>Restarts:</strong> {process_info.restart_count}</p>'

    pid_info = ''
    if process_info.pid:
        pid_info = f'<p class="text-sm text-gray-600"><strong>PID:</strong> {process_info.pid}</p>'

    # Add type/category badge
    type_info = ''
    if metadata.get('type') == 'launch' and metadata.get('filename'):
        type_info = f'<p class="text-xs text-gray-500 mt-1">📄 {metadata["filename"]}</p>'

    # No individual config warnings - the global banner is sufficient
    config_warning = ''

    # Load template and substitute values
    template = _load_template("process_card.html")
    return template.format(
        category_class=category_class,
        process_name=process_info.name,
        description=metadata.get('description', 'No description'),
        type_info=type_info,
        config_warning=config_warning,
        state_class=state_class,
        state_upper=state.upper(),
        category_title=metadata.get('category', 'other').title(),
        pid_info=pid_info,
        started_at=started_at,
        restart_info=restart_info,
        start_disabled_attr="disabled" if start_disabled else "",
        stop_disabled_attr="disabled" if stop_disabled else ""
    )


@app.post("/api/processes/{process_name}/start")
async def api_start_process(process_name: str):
    """Start a specific process"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    success = await server.start_process(process_name)
    if not success:
        raise HTTPException(status_code=500, detail=f"Failed to start {process_name}")

    # Return updated processes HTML
    return await api_render_processes()


@app.post("/api/processes/{process_name}/stop")
async def api_stop_process(process_name: str):
    """Stop a specific process"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    success = await server.stop_process(process_name)
    if not success:
        raise HTTPException(status_code=500, detail=f"Failed to stop {process_name}")

    # Return updated processes HTML
    return await api_render_processes()


@app.get("/api/logs/files", response_class=HTMLResponse)
async def api_get_log_files():
    """Get list of available log files as HTML"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    data = server.get_available_log_files()

    if data.get("error"):
        return HTMLResponse(content=f'<div class="text-red-600 p-4 bg-red-50 rounded-lg">{data["error"]}</div>')

    if not data.get("files"):
        return HTMLResponse(content='<div class="text-gray-600 p-4 bg-gray-50 rounded-lg">No log files found</div>')

    html_parts = ['<div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">']

    for file_info in data["files"]:
        file_size = f"{file_info['size'] / 1024:.1f} KB"
        mod_date = datetime.fromtimestamp(file_info['modified']).strftime("%Y-%m-%d %H:%M:%S")

        html_parts.append(f"""
        <div class="log-file-card bg-white border border-gray-200 rounded-lg p-4 cursor-pointer hover:border-blue-500 hover:shadow-md transition-all"
             onclick="selectLogFile('{file_info['name']}')">
            <h4 class="font-medium text-gray-900 truncate">{file_info['name']}</h4>
            <p class="text-sm text-gray-600 mt-1"><strong>Size:</strong> {file_size}</p>
            <p class="text-sm text-gray-600"><strong>Modified:</strong> {mod_date}</p>
        </div>
        """)

    html_parts.append('</div>')
    return HTMLResponse(content=''.join(html_parts))


@app.get("/api/logs/content/{filename}", response_class=HTMLResponse)
async def api_get_log_content(filename: str, max_lines: int = 200):
    """Get content of a specific log file as HTML"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    data = server.get_log_file_content(filename, max_lines)

    if data.get("error"):
        return HTMLResponse(content=f'<div class="text-red-400">Error: {data["error"]}</div>')

    header = f"""=== {data['filename']} ===
Total lines: {data['total_lines']} | File size: {data['file_size'] / 1024:.1f} KB
Showing last {len(data['lines'])} lines

"""

    content = header + '\n'.join(data['lines'])
    return HTMLResponse(content=f'<pre class="whitespace-pre-wrap">{content}</pre>')


@app.get("/api/config/content", response_class=HTMLResponse)
async def api_get_config_content():
    """Get config file content as HTML textarea"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    content = server.get_config_content()

    # Escape backticks for JavaScript
    escaped_content = content.replace("`", "\\`")

    return HTMLResponse(content=f"""
    <textarea
        id="config-textarea"
        class="w-full h-96 p-4 border border-gray-300 rounded-lg font-mono text-sm resize-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
        placeholder="Configuration file content..."
        onchange="onConfigChange()"
        oninput="onConfigChange()"
    >{content}</textarea>
    <script>
        onConfigContentLoaded(`{escaped_content}`);
    </script>
    """)


@app.post("/api/config/apply")
async def api_apply_config(request_data: dict):
    """Apply config changes"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    content = request_data.get('content', '')

    # Validate first
    validation = server.validate_config_content(content)

    if not validation.valid:
        return JSONResponse(content={
            "success": False,
            "errors": validation.errors,
            "warnings": validation.warnings
        })

    # Apply changes
    success, message = await server.apply_config_changes(content)

    return JSONResponse(content={
        "success": success,
        "message": message,
        "errors": [] if success else [message],
        "warnings": []
    })



@app.post("/api/config/load_url")
async def api_load_config_from_url(request_data: dict):
    """Load config from URL"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    url = request_data.get('url', '')
    if not url:
        return JSONResponse(content={
            "success": False,
            "message": "URL is required"
        })

    success = await server.load_config_from_url(url)

    return JSONResponse(content={
        "success": success,
        "message": "Config loaded from URL successfully" if success else "Failed to load config from URL"
    })


@app.post("/api/alerts/test", response_class=HTMLResponse)
async def api_test_alerts():
    """Test alert system"""
    if not server:
        raise HTTPException(status_code=500, detail="Server not initialized")

    test_result = await server.test_alerts()

    if test_result["success"]:
        return HTMLResponse(content=f'''
        <div class="bg-green-50 border border-green-200 rounded-lg p-4">
            <div class="flex items-center">
                <div class="flex-shrink-0">
                    <svg class="h-5 w-5 text-green-400" viewBox="0 0 20 20" fill="currentColor">
                        <path fill-rule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clip-rule="evenodd" />
                    </svg>
                </div>
                <div class="ml-3">
                    <p class="text-sm text-green-800">✅ {test_result["message"]}</p>
                </div>
            </div>
        </div>
        ''')
    else:
        return HTMLResponse(content=f'''
        <div class="bg-red-50 border border-red-200 rounded-lg p-4">
            <div class="flex items-center">
                <div class="flex-shrink-0">
                    <svg class="h-5 w-5 text-red-400" viewBox="0 0 20 20" fill="currentColor">
                        <path fill-rule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z" clip-rule="evenodd" />
                    </svg>
                </div>
                <div class="ml-3">
                    <p class="text-sm text-red-800">❌ {test_result["message"]}</p>
                </div>
            </div>
        </div>
        ''')


@app.websocket("/ws/logs")
async def websocket_logs(websocket: WebSocket):
    """WebSocket endpoint for real-time log streaming"""
    await websocket.accept()

    if not server:
        await websocket.close(code=1000)
        return

    await server.add_log_connection(websocket)

    try:
        while True:
            # Listen for client messages (log file selection)
            message = await websocket.receive_text()
            try:
                data = json.loads(message)
                if data.get('type') == 'select_log_file':
                    filename = data.get('filename')
                    if filename:
                        # Start monitoring this log file
                        await server.start_log_file_monitor(filename)

                        # Send initial content
                        log_data = server.get_log_file_content(filename, max_lines=200)
                        if not log_data.get('error'):
                            await websocket.send_text(json.dumps({
                                'type': 'initial_content',
                                'filename': filename,
                                'content': '\n'.join(log_data['lines'])
                            }))
            except json.JSONDecodeError:
                logger.warning("Invalid JSON message from WebSocket: %s", message)
    except WebSocketDisconnect:
        await server.remove_log_connection(websocket)


if __name__ == "__main__":
    import argparse
    import uvicorn

    # Parse command line arguments
    parser = argparse.ArgumentParser(description="PhytO-ARM Control Server")
    parser.add_argument("config", nargs="?", help="Config file path (optional)")
    parser.add_argument("--start", type=str, help="Comma-separated list to auto-start (e.g. main,arm_ifcb)")
    parser.add_argument("--port", type=int, default=8080, help="Server port (default: 8080)")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Server host (default: 0.0.0.0)")

    args = parser.parse_args()

    # Get config file from arguments or environment (optional now)
    main_config_file = args.config or os.environ.get('PHYTO_ARM_CONFIG')

    # Set environment variables for app startup
    if main_config_file:
        os.environ['PHYTO_ARM_CONFIG'] = main_config_file
        logger.info("Starting PhytO-ARM Control Server with config: %s", main_config_file)
    else:
        # Remove the env var if it exists
        os.environ.pop('PHYTO_ARM_CONFIG', None)
        logger.info("Starting PhytO-ARM Control Server without config - config must be loaded via web interface")

    # Set auto-start processes if specified
    if args.start:
        os.environ['PHYTO_ARM_AUTO_START'] = args.start
        logger.info("Auto-start processes: %s", args.start)

    uvicorn.run(
        app,
        host=args.host,
        port=args.port,
        log_level="info"
    )
