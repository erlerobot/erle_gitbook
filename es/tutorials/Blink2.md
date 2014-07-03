Parpadeo de un LED en C++
======

Objetivo
-----
En este tutorial explicaremos **cómo hacer un programa en C++ que haga parpadear uno de los LEDs del robot**. El programa se llamará `ledc++`:

``` bash
ledc++ [parámetro]
```
*parámetro* será `on`, `off`, `flash` o `status`.


Material
-----
Para seguir el tutorial necesitas:
- El robot Erle.
- Una tarjeta microSD de 8 GB con la *Distrución Ubuntu Linux* que proporcionamos.
- un conector (macho) USB-to-miniUSB
- Un ordenador con un terminal serial instalado (en Windows puedes usar PuTTy in en *Unixes* `minicom`).


Tutorial
-----

Crea un fichero vacío llamado `ledc++.cpp`:
```
touch ledc++.cpp
```
Edita el fichero (a nosotros nos gusta el editor `vim` pero puedes utilizar cualquier otro que prefieras) y añade el siguiente contenido:

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
Compila el código:
``` bash
g++ ledc++.cpp -o ledc++
```
Finalmente, ejecuta el programa compilado utilizando **uno** de los argumentos [`on`, `off`, `flash` or `status`]:
``` bash
./ledc++ on
```


