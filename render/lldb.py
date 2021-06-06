#! python3
# coding=utf-8

"""
载入命令的方法：command script import ~/Desktop/render_debug/render/lldb.py
"""


def print_vec(debugger, command, result="", internal_dict=""):
    """
    打印 Eigen::Vector3f

    :param debugger: 当前的调试器对象：lldb.SDBDebugger类型
    :param command: 命令的参数：字符串类型
    """
    debugger.HandleCommand(f"print {command}.m_storage.m_data.array")


def print_dir(debugger, command, result="", internal_dict=""):
    """
    打印 Direction 类型
    """
    print_vec(debugger, f"{command}._vec")


def print_ray(debugger, command, result="", internal_dict=""):
    """
    打印自定义的 Ray 类型
    """
    print_vec(debugger, f"{command}._origin")
    print_dir(debugger, f"{command}._direction")


def print_inter(debugger, command, result="", internal_dict=""):
    """
    打印自定义的 Intersection 类型
    """
    debugger.HandleCommand(f"print {command}._happened")
    print_vec(debugger, f"{command}._position")
    print_dir(debugger, f"{command}._normal")
    debugger.HandleCommand(f"print {command}._t_near")


def __lldb_init_module(debugger, internal_dict):
    """
    加载脚本时自动执行
    command script add -f <script-file>.<func-name> <cmd>
    """
    add_cmd_formator = f"command script add -f lldb." + "{0} {0}"
    debugger.HandleCommand(add_cmd_formator.format("print_vec"))
    debugger.HandleCommand(add_cmd_formator.format("print_dir"))
    debugger.HandleCommand(add_cmd_formator.format("print_ray"))
    debugger.HandleCommand(add_cmd_formator.format("print_inter"))
