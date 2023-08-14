
# hybrid_A*

## Table of Contents

- [Overview](#overview)
- [Install](#install)
- [Usage](#usage)
	- [Generator](#generator)
- [Badge](#badge)
- [Related Efforts](#related-efforts)
- [Maintainers](#maintainers)
- [Contributing](#contributing)
- [License](#license)

## Overview


hybrid_A\*算法，应用广泛，在工程上有很多处理的思路。

其中启发式函数的构建主要有四种方式，每种方式都适用于不同的场景，对于复杂的环境下搜索路径，hybrid_A\*感觉还是很好用的。

高飞老师的faster_planner分为前端和后端，其中前端就是使用Hybrid_A*算法，考虑无人机的初始状态和动力学约束（包括运动学和动力学），后端使用基于梯度场的轨迹优化方法。


## Algorithm principle

### basic idea
1. 每个栅格（格子）中只保留一个节点，也就是cost最小的那个节点，cost是广义的，比如能量、时间、距离等综合。





局部规划，基于环境不完全可知，环境部分可知的前提下。也就是全局地图不具有的情况下，依据车上面的传感器来感知周围的局部信息，来进行在线（局部）规划。

在迷宫场景中，可以看到随着车辆的运动，周围在不断的做增量构建,这也就意味着，迷宫中的障碍物是通过车端的传感器实时感知结果得到的。车辆只能看到它周围的环境，随着车辆的持续运动，周围的环境被增量式的构建出来。车辆根据增量构建的场景，实时的调整自身的运动规划策略。



算法流程 workflow
1、搜索空间离散化
2、hybrid A*搜索树拓展
2.1满足车辆运动学约束，不同的车辆模型，满足的运动学约束不一样。
2.2车辆控制空间离散化
2.3对运动空间进行扩展搜索














## Install

This project uses [node](http://nodejs.org) and [npm](https://npmjs.com). Go check them out if you don't have them locally installed.

```sh
$ npm install --global standard-readme-spec
```

## Usage




```sh
$ standard-readme-spec
# Prints out the standard-readme spec
```

<!-- ### Generator

To use the generator, look at [generator-standard-readme](https://github.com/RichardLitt/generator-standard-readme). There is a global executable to run the generator in that package, aliased as `standard-readme`. -->

## Badge





[![zhangs  ](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat-square)](https://github.com/Hailan-9/motion-Planner)




<!-- ## Related Efforts

- [Motion planning and Control of Mobile Robot or Unmanned Vehicle](https://github.com/Hailan-9/motion-Planner) - 💌 . -->

## Maintainers

[@Hailan](https://github.com/Hailan-9).

## Contributing

Feel free to dive in! [Open an issue](https://github.com/Hailan-9) or submit PRs.



### Contributors

This project exists thanks to all the people who contribute.



<!-- ## License -->
