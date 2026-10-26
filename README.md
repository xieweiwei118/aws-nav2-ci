# aws-nav2-ci

GitHub Actions 验证：在云端 Ubuntu 22.04 runner 上 headless 运行
ROS 2 Humble + Gazebo 11 + Nav2 + slam_toolbox + TurtleBot3 Waffle，
加载 AWS RoboMaker Small House / Small Warehouse 仿真世界，
自动完成 SLAM 建图 -> 保存地图 -> AMCL 定位 -> 发送多个导航目标点。

运行入口: `.github/workflows/nav_test.yml`（push 自动触发，也可手动 dispatch 传参）。
