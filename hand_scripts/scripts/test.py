#! /usr/bin/env python


class arm_main:
    def __init__(self):
        list = []
        self.planning_group = "arm_group"
        list.append(self.planning_group)


object_f = arm_main()
object_f.planning_group = "test"
print(object_f.planning_group)