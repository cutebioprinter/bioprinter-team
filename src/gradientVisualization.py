import numpy as np
import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# -------- USER GRADIENT FUNCTION --------
def gradient(x, y, z):

    a = np.clip((x-60)/50,0,1)
    b = np.clip((y-22)/50,0,1)
    c = 2*z
    c = np.clip(c,0,1)

    total = a+b+c
    return np.array([a,b,c])/total


# -------- PARSER --------
def parse_gcode(filepath):

    time_array = []
    position_array = []
    mix_array = []

    x=y=z=0
    feedrate=1000
    current_time=0

    flowrate=0

    max_dt = 0.1

    with open(filepath,"r") as f:

        for line in f:

            line=line.strip()

            if line.startswith(";") or line=="":
                continue

            # -------- detect pump command --------
            pump_match = re.search(r'@pump_pressure\s+\d+\s+([-+]?[0-9]*\.?[0-9]+)', line)

            if pump_match:
                flowrate = float(pump_match.group(1))
                continue


            # -------- feedrate --------
            f_match = re.search(r'F([-+]?[0-9]*\.?[0-9]+)', line)
            if f_match:
                feedrate=float(f_match.group(1))


            # -------- motion --------
            if line.startswith(("G0","G1")):

                new_x,new_y,new_z=x,y,z

                xm=re.search(r'X([-+]?[0-9]*\.?[0-9]+)',line)
                ym=re.search(r'Y([-+]?[0-9]*\.?[0-9]+)',line)
                zm=re.search(r'Z([-+]?[0-9]*\.?[0-9]+)',line)

                if xm: new_x=float(xm.group(1))
                if ym: new_y=float(ym.group(1))
                if zm: new_z=float(zm.group(1))


                dist=np.sqrt((new_x-x)**2+(new_y-y)**2+(new_z-z)**2)

                speed=feedrate/60
                dt = dist/speed if speed>0 else 0

                steps=max(1,int(np.ceil(dt/max_dt)))

                for i in range(1,steps+1):

                    alpha=i/steps

                    interp_x=x+alpha*(new_x-x)
                    interp_y=y+alpha*(new_y-y)
                    interp_z=z+alpha*(new_z-z)

                    interp_time=current_time+alpha*dt

                    # ----- mixing rule -----
                    if flowrate == 0:
                        mix = np.array([0,0,0])
                    else:
                        mix = gradient(interp_x,interp_y,interp_z)

                    time_array.append(interp_time)
                    position_array.append([interp_x,interp_y,interp_z])
                    mix_array.append(mix)

                current_time+=dt
                x,y,z=new_x,new_y,new_z


    return (
        np.array(time_array),
        np.array(position_array),
        np.array(mix_array)
    )


# -------- VISUALIZATION --------
def visualize(position_array, mix_array):

    from mpl_toolkits.mplot3d.art3d import Line3DCollection
    import matplotlib.pyplot as plt
    import numpy as np

    xmin,xmax=60,110
    ymin,ymax=22,72
    zmin,zmax=0,50

    mask = (
        (position_array[:,0]>=xmin)&(position_array[:,0]<=xmax)&
        (position_array[:,1]>=ymin)&(position_array[:,1]<=ymax)&
        (position_array[:,2]>=zmin)&(position_array[:,2]<=zmax)
    )

    pos = position_array[mask]
    mix = mix_array[mask]

    # build line segments between consecutive points
    segments = np.stack([pos[:-1], pos[1:]], axis=1)

    # color each segment by mixture
    colors = mix[:-1]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    lc = Line3DCollection(segments, colors=colors, linewidths=2)
    ax.add_collection(lc)

    ax.set_xlim(xmin,xmax)
    ax.set_ylim(ymin,ymax)
    ax.set_zlim(zmin,zmax)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    plt.show()

    
if __name__ == "__main__":

    time_array, position_array, mix_array = parse_gcode("Shape-Cylinder.gcode")

    visualize(position_array, mix_array)
