import Sofa
import numpy as np
from splib3.numerics import Quat, Vec3
from scipy.spatial.transform import Rotation as R


class MagController(Sofa.Core.Controller):
    '''
    A class that takes the desired magnetic field inputs and calculates the
    torque applied on the magnets of the magnetic instrument. The torques are
    applied to the SOFA mechanical model at every time step.

    :param e_mns: The object defining the eMNS
    :param instrument: The object defining the instrument
    :param T_sim_mns: The transform defining the pose of the sofa_sim frame center in Navion frame [x, y, z, qx, qy, qz, qw]
    :type T_sim_mns: list[float]
    :param field_des: The desired magnetic field (m)
    :type field_des: float
    :param `*args`: The variable arguments are passed to the SofaCoreController
    :param `**kwargs`: The keyword arguments arguments are passed to the SofaCoreController
    '''

    def __init__(
            self,
            e_mns,
            instrument,
            T_sim_mns,
            field_des=np.array([0., 0., 0.]),
            *args, **kwargs):

        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.e_mns = e_mns
        self.instrument = instrument
        self.T_sim_mns = T_sim_mns
        self.field_des = field_des

        # We no longer use a single magnet_moment
        self.BG = [0., 0., 0.]
        self.num_nodes = len(self.instrument.index_mag)

        # Position of the entry point
        self.initPos = np.array([
            self.T_sim_mns[0],
            self.T_sim_mns[1],
            self.T_sim_mns[2]])

    def onAnimateBeginEvent(self, event):
        '''
        Apply the torque on the magntic nodes given a desired field and
        the pose of the nodes.
        '''

        self.num_nodes = len(self.instrument.MO.position)

        for i in range(0, len(self.instrument.index_mag)):
            mag_idx = self.instrument.index_mag[i]
            magnet = self.instrument.magnets[mag_idx]

            # The node index for the magnet in SOFA's MechanicalObject
            node_idx = self.num_nodes - mag_idx - 1
            pos = self.instrument.MO.position[node_idx]
            quat = Quat(pos[3], pos[4], pos[5], pos[6])

            # Update magnetic model with new pose of catheters
            actualPos = (
                np.array([
                    pos[0],
                    pos[1],
                    pos[2]]) +
                self.initPos)  # pose of the tip in Navion frame

            currents = self.e_mns.field_to_currents(
                field=self.field_des,
                position=actualPos
            )
            field = self.e_mns.currents_to_field(
                currents=currents,
                position=actualPos
            )

            self.BG = field

            # Use individual magnet's dipole moment
            m_mag = magnet.dipole_moment
            B = Vec3(
                self.BG[0] * m_mag,
                self.BG[1] * m_mag,
                self.BG[2] * m_mag)
            magnetic_field = B

            # torque on magnet
            r_rot = R.from_quat([quat[0], quat[1], quat[2], quat[3]])
            
            # The magnetic moment direction in local frame, considering polarity
            local_m_dir = [magnet.polarity, 0., 0.]
            X = r_rot.apply(local_m_dir)
            
            T = Vec3()
            T = T.cross(X, magnetic_field)

            # Update forces and torques
            with self.instrument.CFF.forces.writeable() as forces:
                # Apply torque to the specific node that corresponds to the magnet index
                # CFF.indices are 0 to N-1 from the tip (indexFromEnd=True)
                # So we use mag_idx (distance from tip) directly.
                forces[mag_idx] = [0, 0, 0, T[0], T[1], T[2]]

        # visualize magnetic field arrow in SOFA gui
        self.instrument.CFF_visu.forces = [[
            magnetic_field[0], magnetic_field[1], magnetic_field[2], 0, 0, 0]]
