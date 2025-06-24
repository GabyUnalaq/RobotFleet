import numpy as np
import tf

""" Utils module
This module provides utility functions and classes for the PythonRobot framework.

Classes:
- Transform: Represents a 3D transformation including translation and rotation.

Methods:
- quaternion_to_euler: Converts a quaternion to Euler angles (roll, pitch, yaw).
- euler_to_quaternion: Converts Euler angles (roll, pitch, yaw) to a quaternion.
"""


class Transform:
    def __init__(self,
                 translation: np.ndarray = None,
                 rotation: np.ndarray = None,
                 matrix: np.ndarray = None):
        """
        Create a Transform.

        Arguments:
        translation: A 3D vector for translation.
        rotation: A quaternion (4D vector), Euler angles (3D vector), or a rotation matrix (3x3 matrix).
        matrix: A 4x4 transformation matrix.
        """
        if matrix is not None:
            self.matrix = matrix
        else:
            self.matrix = np.eye(4)
            if rotation is not None:
                self.matrix[0:3, 0:3] = Transform._process_rotation(rotation)
            if translation is not None:
                self.matrix[0:3, 3] = translation

    def __eq__(self, other):
        if isinstance(other, Transform):
            return np.allclose(self.matrix, other.matrix)
        return False

    def __str__(self):
        return f"Transform:\n{self.matrix}"
    
    @staticmethod
    def _process_rotation(rotation: np.ndarray):
        if rotation.shape == (4,):  # Quaternion
            return tf.transformations.quaternion_matrix(rotation)[0:3, 0:3]
        elif rotation.shape == (3,):  # Euler angles
            return tf.transformations.euler_matrix(rotation[0], rotation[1], rotation[2])[0:3, 0:3]
        elif rotation.shape == (3, 3):  # Matrix
            return rotation
        else:
            raise ValueError("Rotation must be a quaternion, Euler angles, or a rotation matrix.")

    def apply_on_points(self, points: np.ndarray):
        if points.ndim != 2:
            points = np.array(points, ndmin=2)
        if points.shape[1] > 3:
            raise ValueError("Points must be in shape (N, 2) or (N, 3)")
        if points.shape[1] == 2:
            points = np.hstack((points, np.zeros((points.shape[0], 1))))
        points_h = np.hstack((points, np.ones((points.shape[0], 1))))
        transformed = self.matrix @ points_h.T
        return transformed[:3].T

    def apply_on_quaternion(self, quaternion: np.ndarray):
        if quaternion.shape != (4,):
            raise ValueError("Quaternion must be a 4D vector.")
        transformed_matrix = np.eye(4)
        transformed_matrix[0:3, 0:3] = Transform._process_rotation(quaternion)
        transformed_matrix[0:3, 3] = self.matrix[0:3, 3]
        transformed_matrix = self.matrix @ transformed_matrix
        return tf.transformations.quaternion_from_matrix(transformed_matrix)

    def inverse(self) -> "Transform":
        inv_mat = np.linalg.inv(self.matrix)
        return Transform(matrix=inv_mat)

    def mult(self, other: "Transform") -> "Transform":
        return Transform(matrix=self.matrix @ other.matrix)

    @property
    def translation(self) -> np.ndarray:
        return self.matrix[0:3, 3]

    @property
    def rotation(self) -> np.ndarray:
        return np.array(tf.transformations.quaternion_from_matrix(self.matrix))


def quaternion_to_euler(quaternion: np.ndarray):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).
    """
    if quaternion.shape != (4,):
        raise ValueError("Quaternion must be a 4D vector, not {}".format(quaternion.shape))
    return tf.transformations.euler_from_quaternion(quaternion)


def euler_to_quaternion(euler: np.ndarray):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion.
    """
    if euler.shape != (3,):
        raise ValueError("Euler angles must be a 3D vector.")
    return tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])


if __name__ == "__main__":
    import numpy as np

    # Example usage
    translation = np.array([1, 2, 3])
    rotation = np.array([0, 0, 0, 1])  # Identity quaternion
    transform = Transform(translation=translation, rotation=rotation)

    print(transform)

    points = np.array([[1, 1], [2, 2], [3, 3]])
    transformed_points = transform.apply_on_points(points)
    print("Transformed Points:\n", transformed_points)

    quaternion = np.array([0, 0, 0, 1])
    transformed_quaternion = transform.apply_on_quaternion(quaternion)
    print("Transformed Quaternion:\n", transformed_quaternion)