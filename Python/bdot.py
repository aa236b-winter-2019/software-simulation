import pickle
from igrffx import igrffx

pickle.dump(orbital_position_data,open("adcs\Parameters\orbital_position.txt","rb"))
x=orbital_position_data["x"]
time=orbital_position_data["t"]
print(x[:3,:3])