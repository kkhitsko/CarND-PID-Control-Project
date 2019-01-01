import sys
import matplotlib.pyplot as plt
import numpy as np


def main():

	file = sys.argv[1]	
	with open(file) as f:
            lines = f.readlines()
            print("Lines size: {}".format(len(lines)))
            plt.figure(figsize=(50,2))
            plt.plot(lines)
            #x = np.linspace(0, len(lines), 1)
            #plt.plot(x,0)
	    #plt.ylabel('CTE')
            plt.xlabel('Steps')
            #plt.ylim(-1.0, 1.0 )
            #plt.axis([0,len(lines), None, None ])
	    plt.savefig('out.png')



if __name__ == "__main__":
    main()
