---
title: "MAC Motor Registers & Script Interactions"
---

The MAC400 motor has many different capabilities and can be programmed to use different control modes, operational parameters, and settings. All this information is stored in registers, to which we assign values to change these settings.

#### Understanding Registers

The structure of the registers on the module is fairly straightforward: each register is made up of two 16-bit segments paired together, for 32 bits of information per register. These are binary, so each of the 16 bits can hold a 0 or a 1. This means each 16-bit segment can hold 2^16 different values, and each pair can hold 2^32 different values in total.

Some registers are unsigned, meaning they will always be positive, ranging from 0 (all bits set to 0) to 65,535 (all bits are set to 1). This is fairly straightforward and allows for 65,536 distinct values to be expressed.

In a signed register, the first bit in the register determines if the number is positive or negative. To indicate a negative number, we set the first bit to 1. These registers use Twos Complement, a common method to indicate the value of signed registers. Using Twos Complement, we enter the desired negative value, and it is then inverted such that all values that were 0 are now 1 and all 1s are now 0s. Since the first bit is used to indicate sign, that leave 15 other bits for the value, so the range for a signed register is 2^15 in both the positive and negative directions. It is not critical you understand how Twos Complement works, but it is useful to know when working with binary numbers.

As noted above, the registers on the MAC400 motor are paired 16-bit registers, giving us a total of 32 bits of information per register. For a signed register like position, which can be positive or negative, this means we have a possible range of +/- 2^31 or nearly 2.15 billion values. 

Because the registers are paired, when you read or write to a register, you read or write to two separate 16-bit segments. This means that when querying register 0, you actually read registers 0 and 1. For register 1, you read registers 2 and 3, and so on. *For any given register, the address of the register is twice the register number.*



#### Interpreting Registers

When interpreting register values, we need to consider how the motor stores these values. The paired 16-bit segments are labeled as the Least Significant Bit (LSB) and Most Significant Bit (MSB), with the LSB coming first and MSB coming second. This ends up looking like this:

[LSB,MSB]

When reading registers using these paired bits, we do a bit of math. The value (V) can be expressed in the following way:

V = [MSB x 2^16] + LSB

Because of this, for any value below 2^16, we only use the MSB, and the LSB remains 0. For numbers above that threshold, we use both the MSB and LSB.

When writing to a register, we similarly need to account for the MSB and LSB. Since the MSB will be multiplied by 2^16, we need to know how many times it can be evenly divided by 2^16. The LSB is added on at the end to give us the final value, so we need to know the remainder. Thankfully, the / function in Python gives us the whole number of times a number can be divided by another, and the % function gives us the remainder. For example, when we write a value of 70,000 

70,000/2^16 = 1

70,000%2^16 = 4,4,64

So our final expression is 70,000 = [1 x 2^16] + 4,464

We don't actually have to do this math when entering a value for position, it has been written in to the code. When we read a register, the return is a pair of 16-bit values, and the script must interpret that number to something we can understand. To do this, the script must know the byte order, and byte format. In this case, byte order is known as Little Endian, where the LSB is encoded first and the MSB is encoded last. The script also need to know the format, which changes as we interpret the value.

To start, we have a [LSB,MSB] return. We tell the script to repack this pair using the struct.pack() function, specifying a Little Endian byte order '>' , and a format of two unsigned short bits, 'HH'. This gives us the total value, which we then check for a sign by unpacking the value with Little Endian byte order '>' and a single long singed bit 'l'. In the code, this operation appears like this when reading register 35:

        check = c.read_holding_registers(70, 2)
        (LSB,MSB) = check
        f = struct.unpack('<l',struct.pack('<HH',LSB,MSB))[0]
        print(f)

Remember that register addresses are twice the value of the register number, and we must read the 16-bit register at that address plus the one after it. 

When we want to check a register to see if a specific bit in the 16-bit segment has been written, we can apply a mask. Register 35 is a great example of this, where each bit corresponds to a status or error message. To check if the motor has reached the target position, we can check bit 4, which is the "In Position" bit.

```
mask = 4
position_check = (pos_LSB & 1<<mask)
```

This operation compares the value of the LBS to a 1, which checks if the status or error message is True. If the value is 1, the bit is True, otherwise it is False and the associate status or error message has not been written. By shifting this comparison over 4 spaces, we can check bit 4 to see if the value is 1 or 0. When the comparison returns a 1 in common between the mask and the LSB, we know the "In Position" bit has been written, and the motor has reached the target position.

#### Script & Module Structure

The majority of the code for the winch is enclosed in the class_test.py module. Here, all the necessary functions are in a single Class, which allows them to pass variables to one another. The practical result of this is that multiple functions can access variable generated by another function. 

Broadly, the Winch Class contains 

#### Troubleshooting & Common Problems

If you need help understanding what a function does, what arguments it accepts, or what part of the profile it controls, please refer to the help guide. This can be accessed by running the class_test.py script in the shell, and entering help(Winch) in the console. This will list all the function in the class Winch and the help descriptors for each.

```
>>> help(Winch)
Help on class Winch in module __main__:

class Winch(builtins.object)
|  This class contains all the functions needed to operate the JVL winch
|  and the RBR Maestro with streaming data.
|  
|  Methods defined here:
|  
|  close(self)
|      Changes the motor to passive mode and closes the TCP connection.
|  
|  count_check(self)
|      Returns the value of register 10, P_INST/Instantaneous position. Requires the
|      TCP connection is open or the init() function has been run. Use to check the
|      current position of the motor.
|  
|  error_reset(self)
|      Used to clear any errors in the status/error register after
|      a fatal error has tripped. Writes 0 values to all bits of
|      register 35 and reads the output. If the issue has been resolved
|      and writing was successful, the output should show no fatal
|      errors.
|
```



If you need help with a specific function, you can find that by entering help(Winch.name_of_function) in the shell. 

```
>>> help(Winch.movement)
Help on function movement in module __main__:

movement(self, depth, mode, speed)
    Accepts target depth (negative input), mode (not currently used) and speed
    in RPM. Converts the values to register-accepted values, writes target
    count and speed and switches the motor to position mode to move to the
    target position.
```

Script Error Messages

| Problem                                                      | Solution                                                     |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| The shell returns an error about missing parentheses         | This happens when running Python 3 or later. For these scripts, make sure you are running Python 2.7.14 natively or in a virtual environment |
| The shell returns an error:  NoneType object is not iterable | The script attempts to read a register, but can't connect to the motor, so the register variable is "NoneType". Check that the motor is connected and has the correct firmware installed |
| The shell returns an error about not finding a module        | Make sure the module is installed and the location is added to your path |
| I run the profile_test.py script in the shell but it just returns '>>>' | Make sure you did not accidentally delete any parentheses and are running the correct version of Python |

Issues running the script

| Problem                                             | Solution                                                     |
| --------------------------------------------------- | ------------------------------------------------------------ |
| The script starts, but gets stuck waking the logger | The serial connection to the logger can only handle one connection at a time. This error is caused because a previous connection has not been closed. Check that there isn't a Tera Term or Ruskin window open, and that the last cast ended normally |
| The motor connection in the script fails            | Check that the module has power and has the correct IP address |
| The motor turns the wrong way                       | Check the sign on the specified depth, as it is most likely incorrect |

