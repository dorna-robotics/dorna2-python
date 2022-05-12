import logging
import logging.handlers as handlers
import time

logger = logging.getLogger('my_app')
logger.setLevel(logging.INFO)

logHandler = handlers.RotatingFileHandler('app.log', maxBytes=100, backupCount=1)
logHandler.setLevel(logging.INFO)
logger.addHandler(logHandler)

def main():
    i = 0
    while True:
        time.sleep(0.1)
        logger.info(str(i)+" A Sample Log Statement")
        i += 1
main()