# 前后端连接排查发现

- 项目是 Python/FastAPI（后端）与 Godot（前端）组合。
- 已发现 `start_backend.bat`、`start_godot.bat`、`services/main.py`、`godot_client/project.godot`。
- 工作区存在其他任务的未提交修改，本次需要避免覆盖。
