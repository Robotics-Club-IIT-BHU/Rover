#!/usr/bin/env python3
"""
test_terrain_generator.py

Unit tests for procedural heightmap and SDF world generation in athena_sim.generate_terrain.
"""

import os
import unittest
import xml.etree.ElementTree as ET

from athena_sim.generate_terrain import (
    generate_flat_world_sdf,
    generate_heightmap,
    generate_world_sdf,
    write_grayscale_png,
)


class TestTerrainGenerator(unittest.TestCase):

    def test_heightmap_generation_dimensions_and_normalization(self):
        """Verify heightmap generates 257x257 bytes with normalized values in 0..255."""
        dim = 257
        pixels, meta = generate_heightmap(dim=dim, size_m=120.0)

        self.assertEqual(len(pixels), dim * dim)
        self.assertGreater(meta['elevation_range_m'], 1.0)
        self.assertEqual(min(pixels), 0)
        self.assertEqual(max(pixels), 255)

    def test_png_writer_creates_valid_png_header(self):
        """Verify PNG writer outputs valid 8-byte PNG signature and chunks."""
        test_file = '/tmp/test_terrain_unit.png'
        dim = 33
        pixels, _ = generate_heightmap(dim=dim, size_m=20.0)
        write_grayscale_png(test_file, dim, dim, pixels)

        self.assertTrue(os.path.exists(test_file))
        with open(test_file, 'rb') as f:
            header = f.read(8)
        self.assertEqual(header, b'\x89PNG\r\n\x1a\n')

        # Clean up
        if os.path.exists(test_file):
            os.remove(test_file)

    def test_world_sdf_xml_syntax(self):
        """Verify generated SDF world strings parse as valid XML."""
        world_xml = generate_world_sdf('sproscape_heightmap.png', z_range_m=4.5)
        tree = ET.fromstring(world_xml)
        self.assertEqual(tree.tag, 'sdf')
        self.assertIsNotNone(tree.find('.//world[@name="sproscape"]'))
        self.assertIsNotNone(tree.find('.//heightmap'))

    def test_flat_world_sdf_xml_syntax(self):
        """Verify minimal flat test world parses as valid XML."""
        flat_xml = generate_flat_world_sdf()
        tree = ET.fromstring(flat_xml)
        self.assertEqual(tree.tag, 'sdf')
        self.assertIsNotNone(tree.find('.//world[@name="test_flat"]'))


if __name__ == '__main__':
    unittest.main()
