# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import time
from robomaster import robot


def uwb_callback(sub_info):
    # print(sub_info)
    id, pox_x, pox_y, pox_z, vel_x, vel_y, vel_z, eop_x, eop_y, eop_z = sub_info
    print("uwb info, id:{0} pox_x:{1}, pox_y:{2}, pox_z:{3}, vel_x:{4}, vel_y:{5}, vel_z:{6}, eop_x:{7}, eop_y:{8}, eop_z:{9}".\
          format(id, pox_x, pox_y, pox_z, vel_x, vel_y, vel_z, eop_x, eop_y, eop_z))


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn="3JKDH3B001W80E")# 3JKDH3B00118M5

    ep_ai_module = ep_robot.ai_module

    # 订阅ai模块的事件
    ep_ai_module.sub_uwb_event(callback=uwb_callback)
    time.sleep(150)
    ep_ai_module.unsub_uwb_event()

    ep_robot.close()
