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
            external_magnet,
            instrument,
            T_sim_mns,
            *args, **kwargs):

        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.external_magnet = external_magnet
        self.instrument = instrument
        self.T_sim_mns = T_sim_mns

        # Position of the entry point
        self.initPos = np.array([
            self.T_sim_mns[0],
            self.T_sim_mns[1],
            self.T_sim_mns[2]])

    def onAnimateBeginEvent(self, event):
        '''
        Apply the torque on the magnetic nodes given the external magnet distance decay.
        '''
        self.num_nodes = len(self.instrument.MO.position)
        forces_to_apply = {}

        # Default fallback for visu
        last_b_field = [0, 0, 0]

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
                self.initPos)  # pose of the tip in absolute lab frame

            # Get distance-decaying field from the EM
            B_field = self.external_magnet.get_b_field(actualPos)
            last_b_field = B_field

            # Use individual magnet's dipole moment magnitude
            m_mag = magnet.dipole_moment
            B = Vec3(
                B_field[0] * m_mag,
                B_field[1] * m_mag,
                B_field[2] * m_mag)

            # torque on magnet
            r_rot = R.from_quat([quat[0], quat[1], quat[2], quat[3]])
            
            # The magnetic moment direction in local frame, considering polarity
            local_m_dir = [magnet.polarity, 0., 0.]
            X = r_rot.apply(local_m_dir)
            
            T = Vec3()
            T = T.cross(X, B)

            # Store torque to apply
            forces_to_apply[mag_idx] = [0, 0, 0, T[0], T[1], T[2]]

        # Update forces and torques
        with self.instrument.CFF.forces.writeable() as forces:
            for m_idx, f in forces_to_apply.items():
                forces[m_idx] = f

        # visualize magnetic field arrow in SOFA gui
        self.instrument.CFF_visu.forces = [[
            last_b_field[0]*magnet.dipole_moment, 
            last_b_field[1]*magnet.dipole_moment, 
            last_b_field[2]*magnet.dipole_moment, 0, 0, 0]]
