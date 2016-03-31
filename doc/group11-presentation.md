Main talking points:

* Solonoid kicker
  - First team with the idea
  - Resets by closing the grabbers with ball
* Sideways movement
  - Forward/backward not necessary for defence
  - Makes intercepting and blocking both faster and easier
* Grabbers
  - Fast
  - Open by default, which is easier than many other teams designs and allows
    immediate grabbing
  - Hide away
* General policy
  - Do one thing and ~~do it well~~ not fuck it up.
  - Our robot is simple in design, but works.

Good morning, ladies and gentlemen of the jury. I am the witchfinder general -
I mean a respresentative - of group 11. Our robot is the Scary Tractor Crab. A
theme you will note during this presentation is that a lot of our decisions
focused on using the simplest practical design we could find. This is largely
due to our group having fewer working members than most.

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
of the ball, and close the grabbers.

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









Good morning, ladies and gentlemen of the jury. I will be talking to you about
our robot, the Scary Tractor Crab. While it is certainly scary, and clearly
resembles a tractor (or so I've been told), these are not the core of the
matter. Our robot is a crab. That was clear from very early on, when we decided
that our robot would move sideways, and only sideways. Across the semester
we've been asked countless times how our robot moves forward. It doesn't. Not
directly anyway, it can turn and then move sideways. But for defending - and
our robot is a defender - forwards movement is barely important. The key
actions a defender needs involve blocking, or more abstractly, intersecting a
line. Sideways movement is perfect for this. Take for example, our robot facing
an opposing attacker. If this attacked turns slightly to attempt to shoot past
our robot, it needs only move a short distance to stay in its way.

The sole issue with sideways movement is actually retreiving the ball when it is
in an arbitrary position. To achieve this, we approach it from one side, and
then grab it. Our grabbers are fast, open by default, and tuck away to the
sides. This makes them simple to operate, unlike some designs which require
opening the grabbers while still approaching the ball. Once we have the ball in
our possession, we seek to pass it to our teammate. We kick the ball with a
small solonoid kicker placed at the base of the robot. At this point I'd like
to point out that a fair few teams use a solonoid kicker now, but ours was the
first to have the idea and implement it. It has the advantage of being small in
profile, lightweight, and fairly powerful. Solonoids, being esentially a metal
rod propelled by an electromagnet, need to be reset after use. Typical
approaches involve springs, using gravity was also suggested. Instead, however,
our approach is very simple: The grabbers pushing the ball into the kicker is
what resets it. This is a little counterintuative, as the default state is now
for the solonoid to be extended. In practice, however, this holds up well.

I will not bore you with length discussions of our code, however you may have
already noticed a theme amongst our decisions: They focus on simplicity and
functionality. Our group was unfortunately limited in manpower, and as a result
our decisions were based on making the greatest impact with the resources we
had. At it's core, our robot followed the UNIX philosophy of "do one thing and
do it well"; that one thing being defending, and I hope that we have achieved
it.
