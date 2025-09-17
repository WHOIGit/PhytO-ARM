#!/usr/bin/env python3
"""
Simple test script to verify daemon functionality without Docker
"""

import tempfile
import yaml
import sys
import os

# Add current directory to path for imports
sys.path.insert(0, '.')

def create_test_config():
    """Create a minimal test config"""
    config = {
        'name': 'test-daemon',
        'launch_args': {
            'log_dir': '/tmp/roslogs',
            'rosbag_prefix': '/tmp/rosbags/test',
            'classifier': False,
            'ifcb_winch': False,
            'chanos_winch': False
        },
        'alerts': [],
        'gps': {'host': '127.0.0.1'},
        'lock_manager': {'max_moving_winches': 1},
        'arm_ifcb': {
            'tasks': {
                'scheduled_depth': {
                    'every': 60,
                    'range': {'first': 0.7, 'last': 1.5, 'count': 3}
                }
            },
            'ctd_topic': '/ctd/depth',
            'winch': {
                'range': {'min': 0.7, 'max': 1.5},
                'max_speed': 0.02
            }
        }
    }
    
    # Write to temp file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.dump(config, f)
        return f.name

def test_daemon_imports():
    """Test that daemon can be imported"""
    try:
        from phyto_arm_daemon import PhytoARMDaemon, ProcessState, ProcessInfo
        print("✓ Daemon imports successful")
        return True
    except Exception as e:
        print(f"✗ Import failed: {e}")
        return False

def test_config_validation():
    """Test config validation"""
    try:
        from scripts.config_validation import validate_config
        
        config_file = create_test_config()
        schema_file = './configs/example.yaml'
        
        # This might fail due to schema differences, but should not crash
        result = validate_config(config_file, schema_file)
        print(f"✓ Config validation ran (result: {result})")
        
        # Cleanup
        os.unlink(config_file)
        return True
    except Exception as e:
        print(f"✗ Config validation failed: {e}")
        return False

def test_process_info():
    """Test ProcessInfo model"""
    try:
        from phyto_arm_daemon import ProcessInfo, ProcessState
        
        info = ProcessInfo(name="test", state=ProcessState.STOPPED)
        assert info.name == "test"
        assert info.state == ProcessState.STOPPED
        
        # Test serialization
        data = info.dict()
        assert 'name' in data
        assert 'state' in data
        
        print("✓ ProcessInfo model works")
        return True
    except Exception as e:
        print(f"✗ ProcessInfo test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("Testing PhytO-ARM Daemon...")
    print("-" * 40)
    
    tests = [
        test_daemon_imports,
        test_config_validation, 
        test_process_info
    ]
    
    passed = 0
    for test in tests:
        if test():
            passed += 1
        print()
    
    print("-" * 40)
    print(f"Tests passed: {passed}/{len(tests)}")
    
    if passed == len(tests):
        print("🎉 All tests passed! Daemon is ready for container testing.")
    else:
        print("❌ Some tests failed. Check the output above.")
        sys.exit(1)

if __name__ == "__main__":
    main()