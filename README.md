ur3e_hand_teleop

script:
docker run --runtime=nvidia -it -e DISPLAY -e LOCAL_USER_ID=$(id -9) -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /home/kaz64/study_workspace/toss/toss_eval_tools:/home/user/projects/shadow_robot/src/toss_eval_tools --privileged ros_arduino:latest