import numpy as np
from scipy.spatial.transform import Rotation as R
import Sofa.Core

class ExternalMagnet(Sofa.Core.Controller):
    '''
    A class representing the external permanent magnet (EM).
    It maintains its 3D pose and calculates the magnetic field B(r) 
    using the dipole approximation at any given target point.
    '''
    def __init__(self, root_node, init_pos=[0., 0.10, 0.], init_rot=[0., 0., 0.], 
                 br_n52=1.45, radius=0.025, height=0.030, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        
        self.root_node = root_node
        self.pos = np.array(init_pos)
        
        # Calculate magnet volume and magnitude
        # V = pi * r^2 * h
        volume = np.pi * (radius**2) * height
        self.mu0 = 4 * np.pi * 1e-7
        self.m_epm_magnitude = (br_n52 * volume) / self.mu0
        
        # Initial magnetic moment points downwards (along -Y or whatever init_rot defines)
        # Default: pointing along X axis, then rotated
        base_dir = np.array([1.0, 0.0, 0.0])
        r_init = R.from_euler('xyz', init_rot, degrees=True)
        self.m_epm_dir = r_init.apply(base_dir)
        self.m_epm = self.m_epm_dir * self.m_epm_magnitude

        # Visualize the external magnet in SOFA
        self.em_node = self.root_node.addChild('ExternalMagnet')
        
        q_init = r_init.as_quat()
        
        # Create a visual cylinder (radius 10mm, height 40mm)
        # Orient it so it visually matches the m_epm direction
        self.mech_obj = self.em_node.addObject(
            'MechanicalObject', 
            name='em_dof', 
            template='Rigid3d',
            position=self.pos.tolist() + q_init.tolist(),
            showObject=True,
            showObjectScale=0.08
        )
        
        # We drop OglCylinderModel because 'length' attribute is changed to 'height' 
        # or removed in newer CylinderVisualModel aliases, leading to SceneLoader errors.
        # showObject=True on MechanicalObject is sufficient to see the EM's pose and frame.

    def rotate_magnet(self, axis, angle_rad):
        '''
        Rotate the external magnet in place.
        axis: [x, y, z] vector
        angle_rad: angle in radians
        '''
        r_delta = R.from_rotvec(angle_rad * np.array(axis))
        # Update magnetic moment direction
        self.m_epm_dir = r_delta.apply(self.m_epm_dir)
        self.m_epm = self.m_epm_dir * self.m_epm_magnitude
        
        # Update mechanical object pose for visualization
        curr_pose = self.mech_obj.position.value[0]
        q_curr = R.from_quat(curr_pose[3:7])
        q_new = r_delta * q_curr
        
        new_pose = curr_pose.copy()
        new_pose[3:7] = q_new.as_quat()
        self.mech_obj.position.value = [new_pose]

    def translate_magnet(self, translation_vec):
        '''
        Translate the external magnet in 3D space.
        translation_vec: [dx, dy, dz] in meters
        '''
        self.pos += np.array(translation_vec)
        
        # Update mechanical object pose for visualization
        curr_pose = self.mech_obj.position.value[0]
        new_pose = curr_pose.copy()
        new_pose[0:3] = self.pos.tolist()
        self.mech_obj.position.value = [new_pose]

    def get_b_field(self, target_pos):
        '''
        Calculate the spatial magnetic field at target_pos using the dipole model.
        B = (mu0/4pi) * (3(m·r_hat)r_hat - m) / r^3
        '''
        r_vec = np.array(target_pos) - self.pos
        r_mag = np.linalg.norm(r_vec)
        
        if r_mag < 1e-4:  # Avoid singularity if perfectly overlapping
            return np.array([0.0, 0.0, 0.0])
            
        r_hat = r_vec / r_mag
        b_field = (self.mu0 / (4.0 * np.pi)) * (3.0 * np.dot(self.m_epm, r_hat) * r_hat - self.m_epm) / (r_mag**3)
        return b_field
