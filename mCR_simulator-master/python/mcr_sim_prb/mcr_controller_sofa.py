import Sofa
import numpy as np

from mcr_sim_prb import mcr_mag_controller
from scipy.spatial.transform import Rotation as R


class ControllerSofa(Sofa.Core.Controller):
    '''
    A class that interfaces with the SOFA controller and with the magnetic
    field controller.
    On keyboard events, the desired magnetic field and insertion inputs are
    sent to the controllers.

    :param root_node: The sofa root node
    :param e_mns: The object defining the eMNS
    :param instrument: The object defining the instrument
    :param environment: The object defining the environment
    :param T_sim_mns: The transform defining the pose of the sofa_sim frame center in Navion frame [x, y, z, qx, qy, qz, qw]
    :type T_sim_mns: list[float]
    :param T_sim_mns: The inital magnetic field direction and magnitude (T)
    :type T_sim_mns: ndarray
    '''

    def __init__(
            self,
            root_node,
            external_magnet,
            instrument,
            T_sim_mns,
            *args, **kwargs):

        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.root_node = root_node
        self.external_magnet = external_magnet
        self.instrument = instrument
        self.T_sim_mns = T_sim_mns
        self.listening = True

        self.dfield_angle = 0.

        self.mag_controller = mcr_mag_controller.MagController(
            name='mag_controller',
            external_magnet=self.external_magnet,
            instrument=self.instrument,
            T_sim_mns=self.T_sim_mns,
            )
        self.root_node.addObject(self.mag_controller)

        self.print_insertion_length = False
        self._step_count = 0
        self._last_xtip = self.instrument.IRC.xtip.value[0]

    def onAnimateBeginEvent(self, event):
        self._step_count += 1
        if self._step_count % 100 == 0:
            print(f"ControllerSofa Heartbeat: Simulation is running (Step {self._step_count})")
        
        # 监控 xtip 是否发生异常跳变
        curr_xtip = self.instrument.IRC.xtip.value[0]
        if abs(curr_xtip - self._last_xtip) > 1e-7:
            print(f"[IRC MONITOR] xtip changed: {self._last_xtip:.6f} -> {curr_xtip:.6f} (delta: {curr_xtip - self._last_xtip:.6f}m)")
            self._last_xtip = curr_xtip

        if self.print_insertion_length:
            print(f"Current insertion length: {self.instrument.insertion_len}")

    def onKeypressedEvent(self, event):
        ''' Send magnetic field and insertion inputs when keys are pressed.'''
        # Increment field angle in rad
        dfield_angle = 3.*np.pi/180
        key = event['key']
        # J / L key : z-rotation
        if key.upper() == 'L':
            self.external_magnet.rotate_magnet([0, 0, 1], -dfield_angle)
            print(f"External Magnet rotated (L): {self.external_magnet.m_epm_dir}")
        elif key.upper() == 'J':
            self.external_magnet.rotate_magnet([0, 0, 1], dfield_angle)
            print(f"External Magnet rotated (J): {self.external_magnet.m_epm_dir}")
        # I / K key : x-rotation
        elif key.upper() == 'I':
            self.external_magnet.rotate_magnet([1, 0, 0], -dfield_angle)
            print(f"External Magnet rotated (I): {self.external_magnet.m_epm_dir}")
        elif key.upper() == 'K':
            self.external_magnet.rotate_magnet([1, 0, 0], dfield_angle)
            print(f"External Magnet rotated (K): {self.external_magnet.m_epm_dir}")
            
        # Translation controls (A/D for X, W/S for Y, Q/E for Z)
        d_pos = 0.005 # 5 mm per press for fine control
        if key.upper() == 'A':
            self.external_magnet.translate_magnet([-d_pos, 0, 0])
            print(f"External Magnet translated to: {self.external_magnet.pos}")
        elif key.upper() == 'D':
            self.external_magnet.translate_magnet([d_pos, 0, 0])
            print(f"External Magnet translated to: {self.external_magnet.pos}")
        elif key.upper() == 'W':
            self.external_magnet.translate_magnet([0, d_pos, 0])
            print(f"External Magnet translated to: {self.external_magnet.pos}")
        elif key.upper() == 'S':
            self.external_magnet.translate_magnet([0, -d_pos, 0])
            print(f"External Magnet translated to: {self.external_magnet.pos}")
        elif key.upper() == 'Q':
            self.external_magnet.translate_magnet([0, 0, -d_pos])
            print(f"External Magnet translated to: {self.external_magnet.pos}")
        elif key.upper() == 'E':
            self.external_magnet.translate_magnet([0, 0, d_pos])
            print(f"External Magnet translated to: {self.external_magnet.pos}")
