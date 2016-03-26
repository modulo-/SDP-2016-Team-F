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
