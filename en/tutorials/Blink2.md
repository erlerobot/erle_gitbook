Blink a LED in C++
======

Goal
-----
In this tutorial we will explain how to make a **program in C++ that will switch on and off one of the robot's LED**. The program will be called `ledc++`:

``` bash
ledc++ [parameter]
```
*parameter* will be either `on`, `off`, `flash` or `status`.

Material
-----
In order to follow the tutorial you will need:
- The robot Erle
- An 8 GB microSD card (others can be used) with the *Ubuntu Linux Distribution* that we provide.
- a (male) USB-to-miniUSB
- A computer with a serial terminal installed (you can use PuTTy in Windows or minicom in Unix-based OSs)



Tutorial
-----

Create a blank file called `ledc++.cpp`:
```
touch ledc++.cpp
```
Edit the file (we personally like `vim` but feel free to learn any editor you like) and add the following content:

``` cpp

/** Simple On-board LED flashing program - written by Derek Molloy
    for the ee402 module

    This program uses USR LED 0 and can be executed in three ways:
         makeLED on
         makeLED off
         makeLED flash  (flash at 100ms intervals - on 50ms/off 50ms)
         makeLED status (get the trigger status)
*/

#include<iostream>
#include<fstream>
#include<string>
using namespace std;

#define LED0_PATH "/sys/class/leds/beaglebone:green:usr0"

void removeTrigger(){
   // remove the trigger from the LED
   std::fstream fs;
   fs.open( LED0_PATH "/trigger", std::fstream::out);
   fs << "none";
   fs.close();
}

int main(int argc, char* argv[]){
   if(argc!=2){
	cout << "Usage is makeLED and one of: on, off, flash or status"
<< endl;
	cout << "e.g. makeLED flash" << endl;
   }

   string cmd(argv[1]);
   std::fstream fs;
   cout << "Starting the LED flash program" << endl;
   cout << "The LED Path is: " << LED0_PATH << endl;

   // select whether it is on, off or flash
   if(cmd=="on"){
	removeTrigger();
	fs.open (LED0_PATH "/brightness", std::fstream::out);
	fs << "1";
	fs.close();
   }
   else if (cmd=="off"){
	removeTrigger();
	fs.open (LED0_PATH "/brightness", std::fstream::out);
	fs << "0";
	fs.close();
   }
   else if (cmd=="flash"){
	fs.open (LED0_PATH "/trigger", std::fstream::out);
	fs << "timer";
	fs.close();
	fs.open (LED0_PATH "/delay_on", std::fstream::out);
	fs << "50";
	fs.close();
	fs.open (LED0_PATH "/delay_off", std::fstream::out);
	fs << "50";
	fs.close();
   }
   else if (cmd=="status"){
	// display the current trigger details
	fs.open( LED0_PATH "/trigger", std::fstream::in);
	string line;
	while(getline(fs,line)) cout << line;
	fs.close();
   }
   else{
	cout << "Invalid command" << endl;
   }
   cout << "Finished the LED flash program" << endl;
   return 0;
}
```
Now compile the code:
``` bash
g++ ledc++.cpp -o ledc++
```
Finally, try calling the program with the different arguments [`on`, `off`, `flash` or `status`]:
``` bash
./ledc++ on
```


