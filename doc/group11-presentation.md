Good morning, ladies and gentlemen of the jury. I am the witchfinder general -
I mean a respresentative - of group 11. Our robot is the Scary Tractor Crab. A
theme you will note during this presentation is that a lot of our decisions
focused on using the simplest practical design we could find, in order to focus
on reliability.

To start with the most abstract: Our robot control followes the most basic
pattern. We receive the raw vision feed from the camera, which is then
processed into an abstract representation of the world state. Our planning
system (which I will beifly touch on later) then decides on the course of
action to take, and sends this as an encoded string of commands to the robot
via RF. Finally, the robot executes the commands.

I will focus more closely on the construction of our robot, as this is where
the majority of the interesting decisions where made. To start with, anyone who
observes our robot even briefly will quickly notice that it does not move
forwards and backwards like most other designs. Neither however did we opt for
the flexibility of omni-directional movement. Instead, our robot moves like its
namesake: sideways. This decision was consciously made in the knowledge that
our robot will be defending, and blocking opponents typically involves moving
into their path, or that of the ball. Sideways movement makes this possible
while at the same time, facing them. Further, by focusing only on sideways
movement, instead of implementing omni-directional movement, we can move with
the power of 4 motors in the same direction, giving us a speed advantage over
most robots.

Another key part of our hardware is the mechanism for grabbing the ball, and
when it is in our possession, kicking it. Both the grabbers, and the kicker may
seem underwhelming, however looks can be deciving. The grabbers are small, and
tuck away to the sides. Without knowing they are there, you'd be forgiven for
thinking we didn't have any. This is also their strength, however. Unlike many
other teams, which have to first open their grabbers, approach the ball, and
then close them, our grabbers are open by default. We simply move to one side
of the ball, and close the grabbers. Moving to the correct position in order to
grab the ball was more tricky than it seems, as we cannot blindly charge into
it, but have to approach it from the side.

For kicking we use a solonoid kicker, which in essence is an electromagnet
which propels a small metal rod. By the way, I do feel the need to claim
bragging rights on this one, as our team was the first to use a kicker of this
type, although the concept quickly caught on. The solonoid is essentially the
perfect kicker, it is small, light, and powerful. One interesting note of
operation is that after kicking the metal rod needs to be reset. Typically this
is done with a spring, however in our case, to maximize kicking power, we
instead use the act of the grabbers pushing the ball to the solonoid to also
reset it.

To touch on our planning briefly, the core concept is very simple: We stay on a
semi-circle in our defence area, and remain between the goal and the ball. Of
course our robot will if necessary also leave this position, for instance to
retrieve the ball and pass it to our teammate. I think that this design quite
well illustrates that at it's core, we followed the UNIX philosophy. We did
(mainly) one thing, namely defending our goal, and, as I hope we will be able
to demonstrate, we did it well.
