# La Salle's 2026 Robot
## This code is not offocially supported by wpilib or any vendor because i modified vendor deps. It is not fully idomatic and is on an unstable build of alpha5
```
       .           . .          .        ######=    .   .       .      
         .         .                    :*    .* .   .    .       . .  
       .        .  .    .          ..   ==    .#                       
                .         .#+          .+.     *           :#-  ..    .
 .    . .          .  .. *-  +*     -***-      :+**+  .  =#. .*:. ..   
  .                  . ++      -*+*.     ......      +*+*    . :*  .   
      ..        .     ..++      . . .--------------:          :*       
     .      .   .        .*:  .   .:------------------:      *+        
   .############  *######*        .   --:     ..---------...#.  .      
   -############  *#####+      .     . ::      .----------  :+      .  
   +####********  *####*      :--:      :      .-----------  :*        
.  ####- .      . +####+     -----.     :      .------------  *: ..    
  :####******     +####*      ----      :      .------------. .****=.. 
  *############-  +#####+     .        ::      .-------------        :#
  *-....-*######* +######+      .     :-:     ..-------------         #
 . .       *#####**#####=       .      ::     ..-------------    ..  .#
           .###########* .    ----      :.     .------------. .#####+- 
 -        .-###########+     -----.     :      .-----------:  *: .     
 ##*=.. .:*######=*####* .    .--.      : .    .----------- .:* .  .   
-###############: +#####*   .  .  .    ::      .----------  :* .       
.#############+   +#######.    .     .--:     ..---------. .#     .    
 .    ...    .          ..*.      .:------------------:   .  *-    .   
...       .        ..   +=.   .. . ..--------------.   .      -*       
.                     .*+      =*+*.     ......      +***     .:*      
  .          .          .*-  **.    =*#*=  .   :*#*+     +#  .*-       
     .                    :#=           *.     *         . -#- .       
    .   .       .                .      +:     #    .               .  
                   . .      ...   .     -+    :+      ..          .    
        .    .  .     .   .     . .    ..######-     .      .     .    
```


## Controls
### Driver
- **Run Kicker + Spindexer** = X Button
- **Arm Retract** = DPad Up
- **Arm Deploy** = DPad Down
- **Intake In** = A Button
- **Hood Position to Dashboard Angle** = DPad Right
- **Speed Boost** = Left Bumper
- **Brake Mode** = Right Bumper
- **Run Flywheel to Dashboard Speed (RPS)** = B Button
- **Reset Gyro** = Start Button

### Operator
- **Right Joy X** = Turn Turret
- **Left Joy Y** = Intake Arm
- **Right Trigger** = Shoot At Hub (Vision)
- **Left Trigger** = Outtake Balls
- **Left Bumper** = Aim At Hub (Vision)
- **A Button** = Feed (Vision)

#### Flywheel Speed
- **Low Speed** = DPad Up
- **Medium Speed** = DPad Left
- **Almost High Speed** = DPad Right
- **High Speed** = DPad Down

#### Turret + Flywheel
- **Straight + Flywheel** = Y Button
- **90° Right + Flywheel** = X Button
- **90° Left + Flywheel** = B Button
- **Flywheel Only** = Right Bumper
 --- 
## Autons

#### Short Left Cycle (Hatboro)

- **What was used at Hatboro; shoots from left trench, grabs from the neutral zone, and shoots from left trench again**
  
#### Right Hatboro

- **Mirrored version of Short Left Cycle (Hatboro) for the right side**
  
#### Left Hatboro No Initial Shoot

- **Identical to Short Left Cycle (Hatboro) without the initial shoot from left trench**
  
#### Left Immediate Neutral Cycle + Fill Hopper

- **Immediately goes from left trench to neutral zone, grabs balls, comes back to left trench and shoots, and grabs balls closer to the hub in the neutral zone before time runs out**
  
#### Right Immediate Neutral Cycle + Fill Hopper

- **Mirrored version of Left Immediate Neutral Cycle + Fill Hopper for the right side**
  
#### Stand Still and Shoot From Left Bump

- **Self explanitory**

#### Left Bump Auto (Lehigh)
- TODO
