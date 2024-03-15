import unittest
import os
from config_validation import validate_config

class TestYAMLValidator(unittest.TestCase):

    def setUp(self):
        # Provided default configuration content
        self.default_config_content = """
        top_level:
          second_level:
            third_level: 123 #optional
          second_second_level: #optional
            second_third_level: false
        another_top_level: "hello"
        """

        # Create a temporary default config file for testing
        self.default_config_filename = "temp_default.yaml"
        with open(self.default_config_filename, 'w') as f:
            f.write(self.default_config_content)

    def tearDown(self):
        os.remove(self.default_config_filename)

    def write_temp_user_config(self, content):
        filename = "temp_user.yaml"
        with open(filename, 'w') as f:
            f.write(content)
        return filename

    def test_valid_user_config(self):
        user_config_content = """
        top_level:
          second_level: 
            third_level: 456
          second_second_level:
            second_third_level: true
        another_top_level: "world"
        """
        user_config_file = self.write_temp_user_config(user_config_content)
        self.assertTrue(validate_config(user_config_file, self.default_config_filename))
        os.remove(user_config_file)

    def test_missing_required_key(self):
        user_config_content = """
        top_level:
          second_level: 
        """
        user_config_file = self.write_temp_user_config(user_config_content)
        self.assertFalse(validate_config(user_config_file, self.default_config_filename))
        os.remove(user_config_file)

    def test_wrong_type_user_config(self):
        user_config_content = """
        top_level:
          second_level:
            third_level: "wrongtype" 
        another_top_level: "test"
        """
        user_config_file = self.write_temp_user_config(user_config_content)
        self.assertFalse(validate_config(user_config_file, self.default_config_filename))
        os.remove(user_config_file)

    def test_optional_parent_missing_child(self):
        user_config_content = """
        top_level:
          second_level: 
            third_level: 456
          second_second_level: 
        another_top_level: "world"
        """
        user_config_file = self.write_temp_user_config(user_config_content)
        self.assertFalse(validate_config(user_config_file, self.default_config_filename))
        os.remove(user_config_file)

    def test_missing_optional_parent_and_child(self):
        user_config_content = """
        top_level:
          second_level: 
            third_level: 456
        another_top_level: "world"
        """
        user_config_file = self.write_temp_user_config(user_config_content)
        self.assertTrue(validate_config(user_config_file, self.default_config_filename))
        os.remove(user_config_file)

    def test_only_required_keys(self):
        user_config_content = """
        top_level:
          second_level: {}
        another_top_level: "world"
        """
        user_config_file = self.write_temp_user_config(user_config_content)
        self.assertTrue(validate_config(user_config_file, self.default_config_filename))
        os.remove(user_config_file)

if __name__ == '__main__':
    unittest.main()
