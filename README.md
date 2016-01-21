# SDP-2016-Team-F

## Configuring the SRF stick and the RF chip
Guard character are likely to be # or ~.

Simplest way is to use the graphical XCM (XRF Config Manager). This can be found in the tools directory. Sadly this only works on Windows. This tool can be used both for the stick and arduino RF chip.

The configuration is somewhat persistent.
Once the equipment is configured there is no need to use any code to reconfigure most of it.
It might be good (or necessary) to do some configuring nonetheless, namely, channel (ATCN): 

_Note that the frequency settings do not appear to be persistent (i.e. you’ll need to set them every time you power up)._

ATCN (Channel nubmer)

Group 11 0x60

Group 12 0x67

## Arduino Libraries
To use the arduino code you need to have the libraries used correctly linked. 
Library files can be found in the repo. If linking is broken check your IDE settings to make sure it can find the libs.

## Workflow
Please use the GitHub wiki a lot. If you find something out or tricky and get a solution chances are
that others will have hit the same problem. Please amend or add to the wiki as you see fit.

Issues and tasks will be managed through the GitHub issue tracker.



## Git ##

There are 3 main branches on git:
* master - used for admin and stuff generic to both teams
* group11 - group 11 development
* group12 - group 12 development

These should always be kept in a working state.
### How-To
* always keep main branches in a working state, do not commit any broken code to them!
* master branch is for administrative issues and developments common for both groups (do not break the other group's code! be careful!!!)
* small commits can go directly on the group branches if you are 100% that they're OK (do relevant testing, make sure it's working with everything else)
* larger pieces of work should be done on a branch with a descriptive name
* `git checkout -b <branch>` is used to create a new one locally, and `git push -u origin <branch>` to push it the first time.
* before merging a branch back in, make sure that your code will work with the code currently on master:
  * try rebasing your branch off of the current master: 
     * `git checkout <my-amazing-branch>`
     * `git rebase master`
  * if this gives you get a merge conflict, abort the rebase:
     * `git rebase --abort`
  * and just do a regular merge of master into your branch:
     * `git merge master`
  * if you still get a merge conflict, suck it up and fix it (using a graphical merge tool is strongly encouraged)
  * now you should be at a state where you can test your code with the latest master (having someone else review the code at this stage might catch some errors as well)
  * if everything is alright merge the code into master by doing:
    * `git checkout master`
    * `git merge <my-amazing-branch> --no-ff`
  * the `--no-ff` makes git not fast-forward master even if it can which is useful in case all the changes on a branch need to be reverted at once
  * delete the old branch:
     * `git push origin :<my-amazing-branch>` 

### Random pointers for git:

* when pulling after local commit have been made use `git pull --rebase` which will rebase your local commits on top of the master on origin (makes the tree  look less of a mess)
* use branch names that describe what you are doing; using tokens might be useful if there are a lot of branches: vision/tweaking-calibrations
* run `git remote prune origin` to stop tracking branches that are no longer on origin



