mv ~/.bashrc ~/.bashrc.bak
ln -s ~/AMR-tiago/.bashrc ~/.bashrc
ln -s ~/AMR-tiago/tmux/tmux.conf ~/.tmux.conf

cd ~/AMR-tiago/pymoveit2-4.1.1/
rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"