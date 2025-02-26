from trajopt.utilities.robotOperations import *


# function to a trajectory file (list of waypoints)
def writeTrajectoryFile(st, solution, fname):
    rs = st.getStates(solution)
    print ("Number of spins ", despinSeries(rs))

    # write out things to the "mico-motions file"
    if fname:
        with open(fname, "w") as f:
            length = len(rs)
            for j in range(length):
            # for s in rs:
                # for i, v in enumerate(s):
                for i, v in enumerate(rs[j]):
                    if i > 0:
                        f.write(", ")
                    f.write(str(v))
                f.write("\n")
