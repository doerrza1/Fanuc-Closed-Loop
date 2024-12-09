#!/usr/bin/env python

# Class for roboticstoolbox
import numpy as np
from roboticstoolbox.robot.Robot import Robot

# from spatialmath import SE3


class id7l(Robot):
    """
    Class that imports a Fanuc URDF model

    ``Fanuc()`` is a class which imports a Fanuc robot definition
    from a URDF file.  The model describes its kinematic and graphical
    characteristics.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.URDF.id7l()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, arm is stretched out in the x-direction
    - qr, tucked arm configuration

    .. codeauthor:: Kerry He
    .. sectionauthor:: Peter Corke
    """

    def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "id7l_description/robots/lrmate200id7l.urdf"
        )

        super().__init__(
            links,
            name=name,
            manufacturer="Fanuc",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.qdlim = np.array(
            [2.967, 2.530, 3.717, 3.316, 2.181, 6.283]
            )
        
        self.qz = np.array([0, 0, 0, 0, 0, 0])
        self.qr = np.array([1.57, -0.33, -0.18, 0, -1.11, -1.57]) # Starting position

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    robot = id7l()
    print(robot)

    for link in robot.links:
        print(link.name)
        print(link.isjoint)
        
