import unittest
import numpy as np
import os
import tempfile
from liguard.img.file_io import FileIO

class TestRawImageSupport(unittest.TestCase):
    """Test cases for raw image support in LiGuard."""

    def setUp(self):
        """Set up test fixtures."""
        # Create a temporary directory for test files
        self.test_dir = tempfile.mkdtemp()
        
        # Create a test configuration
        self.test_cfg = {
            'data': {
                'main_dir': self.test_dir,
                'camera_subdir': 'images',
                'start': {'camera': 0, 'global_zero': 0},
                'count': 1,
                'camera': {
                    'img_type': '.raw',
                    'raw_format': 'rgb'
                }
            },
            'pipeline_dir': self.test_dir,  # Add required pipeline_dir
            'threads': {'io_sleep': 0.001}
        }
        
        # Create test images directory
        self.images_dir = os.path.join(self.test_dir, 'images')
        os.makedirs(self.images_dir, exist_ok=True)

    def tearDown(self):
        """Clean up test fixtures."""
        # Close any open FileIO objects first
        if hasattr(self, 'file_io'):
            try:
                self.file_io.close()
            except:
                pass
        
        # Wait a bit for files to be released
        import time
        time.sleep(0.1)
        
        # Try to remove the test directory
        import shutil
        try:
            shutil.rmtree(self.test_dir)
        except PermissionError:
            # On Windows, sometimes files are still locked
            # Try to remove individual files first
            try:
                for root, dirs, files in os.walk(self.test_dir, topdown=False):
                    for name in files:
                        try:
                            os.remove(os.path.join(root, name))
                        except:
                            pass
                    for name in dirs:
                        try:
                            os.rmdir(os.path.join(root, name))
                        except:
                            pass
                os.rmdir(self.test_dir)
            except:
                # If all else fails, just leave it for now
                pass

    def create_test_raw_image(self, width=640, height=480, channels=3, filename='000000.raw'):
        """Create a test raw image file."""
        # Create a simple test image
        test_image = np.random.randint(0, 256, (height, width, channels), dtype=np.uint8)
        
        # Save as raw file
        raw_file_path = os.path.join(self.images_dir, filename)
        test_image.tofile(raw_file_path)
        
        return raw_file_path, test_image

    def test_raw_image_reading(self):
        """Test reading a raw image file."""
        # Create test raw image
        raw_file_path, expected_image = self.create_test_raw_image()
        
        # Test the raw image reading function directly
        self.file_io = FileIO(self.test_cfg)
        
        # Read the raw image
        read_image = self.file_io.__read_raw_img__(raw_file_path)
        
        # Check that the image was read correctly
        self.assertEqual(read_image.shape, expected_image.shape)
        self.assertEqual(read_image.dtype, expected_image.dtype)
        np.testing.assert_array_equal(read_image, expected_image)

    def test_raw_image_reading_with_different_dimensions(self):
        """Test reading raw images with different dimensions."""
        # Test different common dimensions
        test_dimensions = [
            (800, 600, 3),
            (1024, 768, 3),
            (1920, 1080, 3),
        ]
        
        for width, height, channels in test_dimensions:
            with self.subTest(width=width, height=height, channels=channels):
                # Create test raw image
                raw_file_path, expected_image = self.create_test_raw_image(
                    width, height, channels, f'000{width}x{height}.raw'
                )
                
                # Test the raw image reading function
                self.file_io = FileIO(self.test_cfg)
                read_image = self.file_io.__read_raw_img__(raw_file_path)
                
                # Check that the image was read correctly
                self.assertEqual(read_image.shape, expected_image.shape)
                self.assertEqual(read_image.dtype, expected_image.dtype)

    def test_raw_image_reading_grayscale(self):
        """Test reading grayscale raw images."""
        # Update config to use grayscale format
        self.test_cfg['data']['camera']['raw_format'] = 'grayscale'
        
        # Create test grayscale raw image
        raw_file_path, expected_image = self.create_test_raw_image(640, 480, 1, '000001.raw')
        
        # Test the raw image reading function
        self.file_io = FileIO(self.test_cfg)
        read_image = self.file_io.__read_raw_img__(raw_file_path)
        
        # Check that the image was read correctly and converted to RGB
        self.assertEqual(read_image.shape, (480, 640, 3))  # Should be converted to RGB
        self.assertEqual(read_image.dtype, expected_image.dtype)

    def test_raw_image_reading_with_bgr_format(self):
        """Test reading raw images with BGR format."""
        # Update config to use BGR format
        self.test_cfg['data']['camera']['raw_format'] = 'bgr'
        
        # Create test raw image
        raw_file_path, expected_image = self.create_test_raw_image()
        
        # Test the raw image reading function
        self.file_io = FileIO(self.test_cfg)
        read_image = self.file_io.__read_raw_img__(raw_file_path)
        
        # Check that the image was read correctly
        self.assertEqual(read_image.shape, expected_image.shape)
        self.assertEqual(read_image.dtype, expected_image.dtype)

    def test_raw_image_reading_integration(self):
        """Test raw image reading through the main FileIO interface."""
        # Create test raw image
        raw_file_path, expected_image = self.create_test_raw_image()
        
        # Test through the main FileIO interface
        self.file_io = FileIO(self.test_cfg)
        
        # Get the image data
        file_path, read_image = self.file_io[0]
        
        # Check that the image was read correctly
        self.assertEqual(file_path, raw_file_path)
        self.assertEqual(read_image.shape, expected_image.shape)
        self.assertEqual(read_image.dtype, expected_image.dtype)
        np.testing.assert_array_equal(read_image, expected_image)

    def test_raw_image_reading_error_handling(self):
        """Test error handling for invalid raw image files."""
        # Create an invalid raw file that will cause an error
        # Create a file with size that will result in dimensions > 10000
        invalid_raw_path = os.path.join(self.images_dir, '000999.raw')
        
        # Create a file with size that will result in dimensions > 10000
        # For 3 channels, we need file_size > 10000 * 10000 * 3 = 300,000,000 bytes
        # Let's create a file with size that will result in dimensions > 10000
        # A file with 10001 * 10001 * 3 = 300,060,003 bytes will work
        target_size = 10001 * 10001 * 3
        
        # Create the file by writing data in chunks
        with open(invalid_raw_path, 'wb') as f:
            chunk_size = 1024 * 1024  # 1MB chunks
            for i in range(target_size // chunk_size):
                f.write(b'0' * chunk_size)
            # Write remaining bytes
            remaining = target_size % chunk_size
            if remaining > 0:
                f.write(b'0' * remaining)
        
        # Test error handling
        self.file_io = FileIO(self.test_cfg)
        
        with self.assertRaises(ValueError):
            self.file_io.__read_raw_img__(invalid_raw_path)

if __name__ == '__main__':
    unittest.main()
