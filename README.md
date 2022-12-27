# Sorting ROS

A simple pkg that subscribe to a string topic and re-publishes the data as sorted strings.

# Instructions

## Git Practice

1. Fork the repository.
2. Clone your forked repository.
3. Through your terminal go to the cloned repo.
4. Create your branch from the default branch (noetic-devel). `git checkout -b thenameofmynewbranch`
5. Use `git status` to check if you're indeed on the branch `thenameofmynewbranch`
6. Apply your changes, add your files, code some stuff.
7. Add your changes using `git add .` or `git add nameofthespecificfile`. These added files will be the ones commited.
8. Commit your changes (meaning they now exist in the git history of that branch) using `git commit -m "put here a meaningful commit, ie 'added the toggle service'"`
9. Push your changes on the remote. (they'll appear on github, bitbucket, gitlab...). If the branch is new, use `git push -u origin thenameofmynewbranch`. If the branch was already there, simply use `git push`.
10. Changes might have happen remotly by someone else. Use `git pull` to pull these changes.
11. Open a Pull Request on Github/Bitbucket/Gitlab from `thenameofmynewbranch` to the target branch, here `noetic-devel`. Assign a reviewer that will send you change requests. Go back and forth until approved.
12. On approval, you can now merge the PR. In an organized project, the senior will do the merge.
13. There might be CONFLICTS! If the target branch (here `noetic-devel`) has changed since you've made your changes, git may not be able to resolve a merge strategy, and you'll have to manually change the branch.
14. In order to resolve this manually, checkout to the target branch using `git checkout noetic-devel`, then pull the new changes using `git pull`. Head back to your branch using `git checkout thenameofmynewbranch`. Rebase all your commits on top of the target branch. (Meaning it'll cherry pick each commit you've ever made and rebase them on top of the target, you'll resolve the conflict in each commits). Use `git rebase origin/master`.
15. Use this tutorial to resolve conflict: [here](https://linuxpip.org/fix-merge-conflicts-vscode/)

## The ROS Node

### Write a C++ node which:

- Subscribes to the `/hello` (std_msgs/String) topic. 
- Get the string data through a callback, sort the letters alphabetically
- Publish (in the subscription callback) the result to the `/sorted` (std_msgs/String) topic. 
- Exposes a service `/toggle_service` which can be called to toggle the /sorted publisher.
- You should have a seperate package that implements the message. [Here's how to create a msg package](http://wiki.ros.org/msg)

Your file structure shoud look like this:
```
.
├── LICENSE
├── README.md
├── sorting
│   ├── CMakeLists.txt
│   ├── include
│   │   └── sorting.hpp
│   ├── package.xml
│   ├── src
│   │   ├── sorting.cpp
│   │   └── sorting_node.cpp
│   └── test (optional)
│       ├── CMakeLists.txt (optional)
│       └── sorting_test.cpp (optional)
└── sorting_msgs
    ├── CMakeLists.txt
    ├── package.xml
    └── srv
        └── ToggleSorting.srv
```

NB: the C++ node should definitely NOT look like [this](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
It should instead use OOP, like [this](https://roboticsbackend.com/oop-with-ros-in-cpp/)

### Test your C++ node by:

- Publishing to the `/hello` topic using `rosstopic pub /hello "edcba"` or any other string.
- In an other terminal, echo the outflow from the topic `/sorted` using `rostopic echo /sorted`. The result should be ordered alphabetically.
- While the `/sorted` topic is echoed, open a third terminal and call on to your service to toggle the publisher using `rosservice call /toggle_sorting`

## Going deeper: unit testing

Helper library only available in ROS2 for now. So no going deeper here.

## Disclamer

This is not a development package, but a tool to teach some ROS2 stuff.
