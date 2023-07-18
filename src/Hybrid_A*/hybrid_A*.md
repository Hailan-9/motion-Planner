
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















> Your documentation is complete when someone can use your module without ever



~ [Ken Williams, Perl Hackers](http://mathforum.org/ken/perl_modules.html#document)



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
