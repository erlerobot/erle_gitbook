# Electric Power Systems
The following content presents a simplified electric motor model.

### The ideal motor
Lets imagine a simple electric motor which we will call the Ideal Motor.  We call it **ideal** because it has extremely simple characteristics and is 100% efficient in operation.  Our motor has a single quality: for every volt of energy applied to it, the motor will turn at exactly 1000 *Revolutions Per Minute* (RPM).  On 5 volts the motor will turn 5000 RPM, on 25 volts the motor will turn 25000 RPM.  The RPM will always be equal to the amount of volts applied to the motor times the constant of 1000:

|Volts |	RPM|
|------|------------|
|1 volt|	1000 RPM|
|2 volts|	2000 RPM|
|3 volts|	3000 RPM|
|4 volts|	4000 RPM|

Now we need a power source for our motor.  Lets imagine that we have an **Ideal Cell** to match our **Ideal Motor**.  For now, our **Ideal Cell** can be defined by just a single characteristic: the amount of voltage it produces.  Since I am a lover of simple things (and some would say its because likes attract), lets choose a really easy to understand voltage for our **Ideal Cell**.  Each **Ideal Cell** produces exactly 1 volt of electricity.

So now we can make up a pack of Ideal Batteries and attach them to our motor.   Lets see what the results would look like:

| # of Ideal Cells |	RPM |
|---------|--------|
| 1	| 1000 RPM |
| 2	|2000 RPM |
| 3 |3000 RPM |
| 4	|4000 RPM |

---

Careful observers will note that the table above looks a lot like the first table.  Our Ideal Motor and Ideal Cell make life very easy for us. We can almost use the words "cell" and "volts" interchangeably because of our contrived values.  Naturally, the real world isn't quite this simple, but its not entirely unlike our Ideal world either.  Keep that in mind as we go along.

----

The relationship between volts (cells) and RPM for our Ideal power system can work backwards as well as it does forwards.  So if I measure that my Ideal Motor is spinning at 4000 RPM I can bet my sweet patootie that the input voltage is precisely 4 volts.  That in turn means that 4 cells are being used.  Simple so far.

But!  You may notice that our motor is just happily spinning away and doing nothing!  We need to add something to the output shaft so that it can twirl around and move lots of air.  So lets put a propeller on our motor and watch what happens to the RPM.  In fact, lets compare two different propellers with the same motor given various cell counts.

If you are really new to R/C then you might need to know how propellers are specified.   Each propeller has a *diameter* and a *pitch*. The word "pitch" can be a little confusing. It refers to the distance the propeller would travel forward in one revolution in a perfect medium.  The higher the pitch the more angled the blades of the propeller are and the farther it would travel in a single revolution.  A high-pitch prop is usually used on a fast airplane while a low pitch prop is usually used on a slow airplane.

In the US propellers are specified in inches.   Lets use two propellers for our example.
- One will have a 5 inch diameter and a 5 inch pitch, which we'll designate as 5x5.
- The other will have a 12 inch diameter and an 8 inch pitch, which we'll designate as 12x8.

|# of Ideal Cells|	Propeller|	RPM|
|-----|-----|
|1	|5x5	|1000 RPM|
|2|	5x5|	2000 RPM|
|3|	5x5|	3000 RPM|
|4|	5x5|	4000 RPM|

|# of Ideal Cells|	Propeller|	RPM|
|-----|-----|
|1|	12x8|	1000 RPM|
|2|	12x8|	2000 RPM|
|3|	12x8|	3000 RPM|
|4|	12x8|	4000 RPM|

Nothing in the above table should surprise you because our Ideal Motor always turns `1000 RPM` for every volt regardless of what kind of load is placed on the output shaft.  This is a crucial point and its real-world analogy is one of the hang-ups that keeps many beginners from understanding electric power completely.

Of course we must also realize that it takes far more energy to spin a `12x8` propeller at `4000 RPM` than it does to turn a 5x5 propeller at the same rate.  There must be something missing from our simplified motor model.

Indeed, the thing we are missing is called current.  Current is the other half of the energy equation.  Sadly, we can go no further without introducing some kind of formula into the discussion:
```
Watts = Volts x Amps
```
---

Note: the formula is expressed by its units, the proper physical formula is `P = I x V`

---

Lets put Watts to work right now, in fact.  Remember the last chart that showed our Ideal Motor mated with a variable number of Ideal Cells to turn two different propellers?  Remember how the RPM was always dependent solely on how many volts were applied to the motor, even though it takes a lot more effort to turn a big prop than it does a small one?  Lets use watts to show just how much effort is involved:

---

Note: this values are not necessarily real

---

| |	5x5	|12x8|
|---|----|----|
|1000 RPM|	1 watt|	10 watts|
|2000 RPM|	4 watts|	40 watts|
|3000 RPM|	10 watts|	100 watts|
|4000 RPM|	25 watts|	250 watts|

Note that it takes a lot more power (energy) to turn a 12x8 prop at 4000 RPM than it does to spin a 5x5 prop.  Also it takes more than twice as much power to spin a prop at twice as much RPM.

Since we now know how to express power `watts = volts x amps`, we can pull an example out of the table above and see what is going on with our Ideal Motor. Lets concentrate on the table entry which shows that it takes `100 watts` to spin a `12x8` propeller at `3000 RPM`.  Since we know that watts is the product of volts and amps this means that we would need a combination like the following examples:

- `100 watts = 1 volt x 100 amps`
- `100 watts = 2 volts x 50 amps`
- `100 watts = 3 volts x 33 amps`
-    ...and on and on...

So we can actually get 100 watts in many different ways.  Our motor, though, is hard-wired to give us exactly `1000 RPM per volt` regardless of anything else.  Since we are trying to spin our propeller at 3000 RPM, this means we need 3 volts.  And, therefore, our motor must be drawing 33 amps of current from our Ideal Cells.  This is an inescapable conclusion.  For our motor to turn 3000 RPM with that prop it must have 3 cells and it must draw 33 amps because 100 watts are required to perform that demanded task.

Lets take a look at one of our tables and fill in values for current and watts as appropriate:

|# of Ideal Cells|	Current	|Propeller|	RPM|	Power|
|----|----|----|----|----|
|1|	1|	5x5|	1000|	1 watt|
|2|	2|	5x5|	2000|	4 watts|
|3|	3|	5x5|	3000|	10 watts|
|4|	6|	5x5|	4000|	25 watts|

|# of Ideal Cells|	Current	|Propeller|	RPM| Power|
|----|----|----|----|----|
|1|	10|	12x8|	1000|	10 watts|
|2|	20|	12x8|	2000|	40 watts|
|3|	33|	12x8|	3000|	100 watts|
|4|	63|	12x8|	4000|	250 watts|

So here is what we've learned so far:

- Our motor turns exactly 1000 RPM for every volt regardless of load
- Our motor draws the current necessary to make the watts of electrical energy equal the watts of power it takes to turn the prop at the rate demanded by the voltage.
- Watts is the product of volts and amps
- A big propeller requires a lot more power to spin at a specific RPM than a small propeller does
- Given a fixed propeller and motor, increasing voltage will increase current at an exponential rate.


### Sources

- http://www.rebelpeon.com/quadcopter-research-and-education/
- http://www.rcgroups.com/forums/showthread.php?t=333326
