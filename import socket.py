import socket
import pickle
import StringIO
example_dict={1:"6",2:"2",3:"f"}
xyz = StringIO.StringIO()
pickle_out = open{"dict.pickle","w"}
pickle.dump(example_dict, xyz)
print xyz