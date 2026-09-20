import unittest

import parse_stp_full2 as step


class StepParserTests(unittest.TestCase):
    def test_multiline_strings_and_complex_entities(self):
        text = """ISO-10303-21;
DATA;
#1=PRODUCT('a;b','it''s', '',());
#2=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));
#3=AXIS2_PLACEMENT_3D('',#4,#5,#6);
#4=CARTESIAN_POINT('',(1.,\n2.,3.));
ENDSEC;
END-ISO-10303-21;"""
        entities = step.parse_entities(text)
        self.assertEqual(len(entities), 4)
        self.assertEqual(entities[3][0], 'AXIS2_PLACEMENT_3D')
        self.assertIn('SI_UNIT', entities[2][1])
        self.assertEqual(step.entity_refs("'ignore #99',#4"), [4])

    def test_product_shape_and_repeated_occurrences(self):
        text = r"""
#1=PRODUCT('leg','leg','',());
#2=PRODUCT_DEFINITION_FORMATION('','',#1);
#3=PRODUCT_DEFINITION('','',#2,#90);
#4=PRODUCT_DEFINITION_SHAPE('','',#3);
#5=SHAPE_DEFINITION_REPRESENTATION(#4,#6);
#6=SHAPE_REPRESENTATION('',(#80),#90);
#7=SHAPE_REPRESENTATION_RELATIONSHIP('','',#6,#8);
#8=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#81),#90);
#9=NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','',
'\X2\817F90E8\X0\',#30,#3,$);
#10=NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','','same',#30,#3,$);
#11=SHAPE_REPRESENTATION_RELATIONSHIP('','',#6,#12);
#12=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#82),#90);
"""
        entities = step.parse_entities(text)
        shapes, occurrences = step.build_links(entities)
        self.assertEqual(shapes[3], [8, 12])
        self.assertEqual(len(occurrences), 2)
        self.assertEqual(occurrences[0]['name'], '腿部')
        self.assertEqual(occurrences[0]['child_pd'], 3)
        self.assertNotEqual(occurrences[0]['id'], occurrences[1]['id'])

    def test_vertex_bounds_exclude_surface_control_points(self):
        entities = step.parse_entities("""
#1=MANIFOLD_SOLID_BREP('',#2);
#2=CLOSED_SHELL('',(#3,#4));
#3=VERTEX_POINT('',#5);
#4=VERTEX_POINT('',#6);
#5=CARTESIAN_POINT('',(1.,2.,3.));
#6=CARTESIAN_POINT('',(4.,6.,8.));
#7=CARTESIAN_POINT('',(55000.,55000.,55000.));
""")
        points = step.vertex_points([1], entities)
        self.assertEqual(sorted(points), [(1., 2., 3.), (4., 6., 8.)])


if __name__ == '__main__':
    unittest.main()
