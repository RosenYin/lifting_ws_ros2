from distutils.core import setup
from Cython.Build import cythonize
import os

# 指定输出目录
build_dir = "build"

# 确保输出目录存在
os.makedirs(build_dir, exist_ok=True)

# 列出需要编译的文件并指定输出目录
ext_modules = cythonize([
    "lifting_ctrl_service_node_850pro.py",
    "lifting_ctrl_service_node.py",
    "lifting_motor_ctrl_850pro.py",
    "lifting_motor_ctrl.py",
    "serial_encapsulation.py",
    "json_config.py",
    "crc_check.py"
], build_dir=build_dir)

setup(
    name="my_project",
    ext_modules=ext_modules,
    script_args=['build_ext', '--inplace'],
    options={'build_ext': {'build_lib': build_dir}}
)
