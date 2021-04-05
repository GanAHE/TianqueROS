import xml.etree.ElementTree as ET
import codecs

def indent(elem, level=0):
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

class XMLOutput():

    def __init__(self, groups, board):
        xml_parameters = ET.Element("airframes")
        xml_version = ET.SubElement(xml_parameters, "version")
        xml_version.text = "1"
        xml_version = ET.SubElement(xml_parameters, "airframe_version_major")
        xml_version.text = "1"
        xml_version = ET.SubElement(xml_parameters, "airframe_version_minor")
        xml_version.text = "1"
        for group in groups:
            xml_group = ET.SubElement(xml_parameters, "airframe_group")
            xml_group.attrib["name"] = group.GetName()
            xml_group.attrib["image"] = group.GetImageName()
            for param in group.GetParams():

                # check if there is an exclude tag for this airframe
                excluded = False
                for code in param.GetArchCodes():
                    if "CONFIG_ARCH_BOARD_{0}".format(code) == board and param.GetArchValue(code) == "exclude":
                        excluded = True

                if not excluded:
                    #print("generating: {0} {1}".format(param.GetName(), excluded))
                    xml_param = ET.SubElement(xml_group, "airframe")
                    xml_param.attrib["name"] = param.GetName()
                    xml_param.attrib["id"] = param.GetId()
                    xml_param.attrib["maintainer"] = param.GetMaintainer()
                    for code in param.GetFieldCodes():
                        value = param.GetFieldValue(code)
                        xml_field = ET.SubElement(xml_param, code)
                        xml_field.text = value
                    for code in param.GetOutputCodes():
                        value = param.GetOutputValue(code)
                        valstrs = value.split(";")
                        xml_field = ET.SubElement(xml_param, "output")
                        xml_field.attrib["name"] = code
                        for attrib in valstrs[1:]:
                            attribstrs = attrib.split(":")
                            xml_field.attrib[attribstrs[0].strip()] = attribstrs[1].strip()
                        xml_field.text = valstrs[0]

        indent(xml_parameters)
        self.xml_document = ET.ElementTree(xml_parameters)

    def Save(self, filename):
        self.xml_document.write(filename, encoding="UTF-8")
