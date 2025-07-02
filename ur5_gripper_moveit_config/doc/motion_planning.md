# 如何进行精准运动规划
> 是为了让UR机器人尽量走捷径，不要走那些远路，下面是个明显对比

## 当前效果

| UR Robot                        | Panda Robot                        |
| ------------------------------- | ---------------------------------- |
| ![ur](images/ur_robot_move.gif) | ![panda](images/panda_robot_move.gif) |

同样follow这个[tutorial](https://moveit.picknik.ai/main/doc/examples/motion_planning_api/motion_planning_api_tutorial.html)来做的，但是UR明显在走远路，但是panda的轨迹就很好，非常的捷径。

我需要搞清楚这是为什么。

现在已经问了一个问题：https://robotics.stackexchange.com/questions/113562/issues-with-motion-planning-path-on-ur-robot-using-moveit-api-tutorial

有一个人回答说让我做插值，但是效果仍然不行，做了插值，每个姿态之间的距离差异很小了，还是会绕远路。

## 问题回答
轨迹相较于Panda而言那么长那么扭曲大致上是由于以下几个原因导致的：
**1. 使用的planner是 RRTConnect。**
  
- 这个规划器并不擅长输出最优路径，所以走冤枉路是在所难免的。

- 虽然我在RVIZ里面尝试设置了不同的规划器，但是它并不会影响程序所使用的规划器，只会影响在rviz中做plan的时候用的规划器。
**2. Panda Robot更加灵活**
- Panda robot用的也是RRTConnect规划器，它能够得到较为短，平滑的规划器可能是因为它本身自由度是7个，更加灵活
**3. 目标姿态不友好**
- tutorial中设置的目标姿态对于Panda的工作空间来说就是一个比较舒服易达的姿态，但是对于UR来说并不是很友好
**4. 调用方法问题 （猜测）**
出现上面这个问题时follow的tutorial是[这个](https://moveit.picknik.ai/main/doc/examples/motion_planning_api/motion_planning_api_tutorial.html)。仔细读的话发现它是一个**比较偏底层的api展示**。他在进行运动规划的时候，并没有使用最常用的 `move_group_interface`，而是用了一个叫做 `planning_interface`的接口。目前我没有找到给它设置其他规划器，比如RRTstar的方法。

```cpp{.line-numbers}
// 可以同通过这个方法设置，但是提示找不到对应的规划器 （即使ompl_planning.yaml被正确设置了）。最终还是给切换回RRTConnect了
req.planner_id = "RRTstar";
req.group_name = PLANNING_GROUP;
req.goal_constraints.push_back(pose_goal);
planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code);
context->solve(res);
```

我自己实测，设置同样的目标位姿，同样使用`RRTConnect`规划器，通过`move_group_interface`进行路径规划的结果也要平滑的多。

> Follow[这个tutorial](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html)

如下图。不知道是不是因为有什么后处理的算法导致的区别，现在先不研究了。因为我们用的最多的还是 `move_group_interface`，底层的了解一下就好，没必要完全搞透。

![a](images/move_group_interface_rrt_connect.gif)

解决的策略对应如下：
**1. 使用更好的规划器**
比如RRTstar, PRMstar等等，实测是能够给出更加平滑的路径。

> 但如上所述，底层的那个调用没有办法修改更高级的规划器

**2. 设定更适合UR机器人的目标姿态**
这个在demo里面可以随意调整，但是实际运行的时候肯定是需要什么姿态就是什么姿态，没有什么适合不适合。

## 这一调试过程中遇到的问题和经验总结

### 1. Moveit Setup Assistant 并未生成 `ompl_planning.yaml`文件

MSA 2.0 并不会像1那样自动生成该文件，参考[[1]](https://github.com/moveit/moveit2/issues/2256) [[2]](https://github.com/moveit/moveit2/issues/2265)。

这个文件是用来在moveit 里面设置OMPL里面的规划器的，如果不设置就无法使用，在plan的时候rviz中的ompl planner下面会不显示规划器，只有一个unspecified.

但是这个文件只是配置而已，ompl都是安装了的。

与Moveit1所使用的ompl_planning.yaml文件不同，这文件的开头需要加上

```yaml{.line-numbers}
planning_plugin: ompl_interface/OMPLPlanner
request_adapters: >-
  default_planner_request_adapters/AddTimeOptimalParameterization
  default_planner_request_adapters/ResolveConstraintFrames
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStateCollision
  default_planner_request_adapters/FixStartStatePathConstraints
```

这个在上面的两个回答里面都有说明。

其余的部分都和Moveit1基本一样。

这里也说明一下文件的使用逻辑。OMPL是一个开源的运动规划仓库，在安装moveit或者ROS的时候就是已经安装了的，里面就包好 RRTConnect, RRTstar, PRMstar等等规划算法。跟有没有 `ompl_planning.yaml` 这个文件是无关的。这个文件是moveit所需要的，它用来设定我们在本次的moveit launch中，会载入以及使用哪些规划算法。对于大部分的机器人而言，前面的部分都是一样的，只是在最后需要根据规划组设定一下。

### 2. 添加 `ompl_planning.yaml`后机器人规划失败
我原先没有这个文件，一直都是unspecified，但是大部分规划也能成功。添加了这个之后，我自己写的 robot_function_test node反而规划不成功了。甚至在rviz里面规划都会失败。

会报这个错误：
```bash{.line-numbers}

[move_group-4] Skipping adapter instead.
[move_group-4] [WARN] [1730596255.145292513] [ompl]: ./src/ompl/base/goals/src/GoalLazySamples.cpp:129 - Goal sampling thread never did any work. Space information not set up.
[move_group-4] [ERROR] [1730596255.145507179] [moveit.planning_request_adapter]: Exception caught executing adapter 'Fix Start State In Collision': bad lexical cast: source type value could not be interpreted as target
[move_group-4] Skipping 
```

后面发现这是一个乌龙，规划失败是因为当时设置的目标姿态确实是达不到的。通过在 `ompl_planning.yaml` 中设置默认规划确实可以改变程序所使用的默认规划器。

### 3. 程序上在哪里设置和切换不同的planner
rviz里面的规划可以通过下拉菜单来选择，程序上呢，如果只是能够通过修改 ompl_planning.yaml 来改的话，运行过程中如何切换呢？
- 通过在 `ompl_planning.yaml` 中设置启动时的默认规划器
- 程序启动后，可以获取 `move_group` 对象，通过想要使用的规划器的名称动态设置。

```cpp {.line-numbers}
moveit::planning_interface::MoveGroupInterface move_group(motion_planning_api_tutorial_node, PLANNING_GROUP);

move_group.setPlannerId("RRTstar");{.line-numbers}
```

### 4. tutorial跟实际规划的不同之处
实际规划用的更多的是封装好的move_group_interface。现在我也在测试。

当时提出这个问题是因为正在跑的是[这个](https://moveit.picknik.ai/main/doc/examples/motion_planning_api/motion_planning_api_tutorial.html)api的tutorial, 它使用的是底层方法 `planning_interface`，我们实际用的时候还是用 `move_group_interface` 方法，跟我现在写的是一样的。

### 5. 为何我的姿态无法在预览直接走
现在测试move_group_interface这个，发现panda可以不考虑实际位置，直接从预览位置进行规划模拟，我的上来就会规划失败，明明起始位置可以到的。

> 破案了，是因为我设置的目标姿态超出了我所设置的joint limits的范围，这个在erorr message里面是可以看到的。

这个实际上是一个乌龙。

当时提出这个问题是因为正在跑的是 move_group_interface tutorial的约束轨迹部分。约束轨迹的起点设置的并不是当前状态，然后要求机器人在满足约束的情况下走到另一个姿态。如下两图所示


|![a](images/failed_constraint_path.gif)|![b](images/success_constraint_path1.gif)|
|-|-|

当时我点下开始规划的时候，panda机器人会出现上面两图所示的预览动作和轨迹，而我的啥都没有，机器人整体静止什么都没做。

后面排查到原因是因为当时设置的目标姿态对于我的机器人来说是不可达的，因为我设置了joint_limits。后面取消掉就可以了，得到了上面的两个动图。

这里还需要说明的一点是，两个动图中一个显示了轨迹，一个没有显示轨迹。区别在于显示轨迹的那个是成功的规划，也可以看到他全程都保持了末端竖直向上的约束。另外一个是一个失败的结果，所以轨迹没有显示出来。

实测发现，这种轨迹约束不满足或者中途发生碰撞都会导致规划失败的情况下仍然给预览，只是轨迹给不出来而已。

### 6. Move group interface的demo是怎么只规划不走的
他只调用了move_group的plan方法，然后把plan得到的结果使用visual tools可视化了出来，这个我也可以做。