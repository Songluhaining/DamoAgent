"""仿真层：把 robtarget 送进虚拟控制器执行，取回运动学结果。

  rapid.py      robtarget 数组 → RAPID 模块文本（填 damo_routine）
  rws.py        RWS 2.0 客户端：加载模块、启停执行、读关节/节拍/报警。桩模式可空跑。
  collision.py  RobotStudio Add-in 的 HTTP 客户端：几何碰撞。桩模式可空跑。
"""
