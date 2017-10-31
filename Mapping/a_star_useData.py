import numpy as np
import a_star_final_withOutput as pf

#mapFileName = "mit_map_1.txt"

mapFileName = "PracticeMap2.txt"

fileName = mapFileName + "output" + ".npz"

with np.load(fileName) as X:
    init_heading, path_total, bounds, boundaryGrid, maxX, maxY = [X[i] for i in ('init_heading','path_total','bounds','boundaryGrid', 'maxX', 'maxY')]

init_heading = float(init_heading)
path_total = list(path_total)
maxX = int(maxX)
maxY = int(maxY)


new_path = pf.get_clean_path(path_total[0], path_total[len(path_total)-1], boundaryGrid, bounds, maxY)

##new_path = pf.get_clean_path((self.x,self.y), self.targetCoord, boundaryGrid, bounds, maxY)
