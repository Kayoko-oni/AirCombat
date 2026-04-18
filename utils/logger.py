import logging
""" 导入Python标准库中的日志模块 """


def get_logger(name: str):
    logger = logging.getLogger(name)
    #logging.getLogger(name)会返回一个指定名称为name的logger对象, 如果对应名称已经存在logger, 则返回已存在的, 否则新建一个"""
    if not logger.handlers:
        handler = logging.StreamHandler()
        #StreamHandler() 将日志输出到流 """
        formatter = logging.Formatter("[%(levelname)s] %(asctime)s %(name)s: %(message)s")
        handler.setFormatter(formatter)
        #设置日志的格式, 包括事件级别, 时间戳, 日志名称, 日志信息"""
        logger.addHandler(handler)
        #将handler添加到logger, 这样logger在记录日志是也会将日志输出到控制台"""
        logger.setLevel(logging.INFO)
        #设置日志等级, 只有等级超过info的日志内容才会被处理 """
        #返回日志实例 """
    return logger
