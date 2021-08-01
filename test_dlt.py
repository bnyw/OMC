from coor import EuclidDistance, load_from_file
from dlt import DLT

dlt = load_from_file("param.dlt")

topLeft = dlt.getXYZ((848, 144), (786, 57))
topRight = dlt.getXYZ((1397, 160), (995, 513))
bottomLeft = dlt.getXYZ((850, 576), (591, 293))
bottomRight = dlt.getXYZ((1404, 573), (726, 712))

print(EuclidDistance(topLeft, bottomRight), EuclidDistance(bottomLeft, topRight))
print(EuclidDistance(topLeft, topRight))
print(EuclidDistance(topRight, bottomRight))