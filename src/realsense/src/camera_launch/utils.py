from pathlib import Path
import os

home = os.environ["HOME"]
HISTORY_DB = "history.db"

def get_turtle_ip():
    curr_dir = Path(__file__).parent
    history_db_path = curr_dir.joinpath(HISTORY_DB)
    if not history_db_path.exists():
        history_db_path_link = curr_dir.joinpath(history_db_path.name)
        src_path = Path(home).joinpath("turtlebot3_network", HISTORY_DB)
        if not src_path.exists():
            raise Exception(f"History DB not found in {str(history_db_path)}\n" + \
                            f"did you place the folder in in the HOME ({home})?")
        os.symlink(src_path, str(history_db_path_link))
    
    with open(history_db_path) as fil:
        return fil.readline()