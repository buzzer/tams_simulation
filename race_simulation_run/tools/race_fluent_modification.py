#! /usr/bin/env python
import yaml
import random
import roslib

class Fluent(yaml.YAMLObject):

    yaml_tag='!Fluent'

    def __setstate__(self, state):
        """
        PyYaml does not call __init__. this is an init replacement.
        """
        self.properties = []
        self.Class_Instance = state['Class_Instance']
        # fix for bug:
        if type(self.Class_Instance[0]) == type(False):
            self.Class_Instance[0] = '"On"'
        if self.Class_Instance[0] == 'On':
            self.Class_Instance[0] = '"On"'
        self.StartTime = state['StartTime']
        self.FinishTime = state['FinishTime']

        for preProp in state['Properties']:
            self.properties.append(Property(preProp[0], preProp[1], preProp[2]))

    def toYamlString(self, line_break='\n'):
        yString = '' 
        yString += '!Fluent' + line_break
        yString += '{:<16} {:<20}'.format('Class_Instance: ', '[' + self.Class_Instance[0] + ', ' + self.Class_Instance[1] + ']') + line_break
        yString += '{:<16} {:<20}'.format('StartTime: ', str(self.StartTime)) + line_break
        yString += '{:<16} {:<20}'.format('FinishTime: ', str(self.FinishTime)) + line_break

        if len(self.properties) > 0:
            yString += 'Properties:' + line_break
            for prop in self.properties:
                yString += prop.toYamlString()
        else:
            yString += 'Properties: []' + line_break

        return yString

    def __str__(self):
        return self.toYamlString()

class Property:

    def __init__(self, role_type, filler_type, role_filler):
        self.role_type = role_type
        self.filler_type = filler_type
        self.role_filler = role_filler

        for key, value in self.__dict__.items():
            if type(value) == type(False):
                setattr(self, key, 'On')
            #if value == 'On':
            #    setattr(self, key, '"On"')
        

    def toYamlString(self, line_break='\n'):
        return '  -  [{}, {}, {}]'.format(self.role_type, '"' + self.filler_type + '"', self.role_filler) + line_break 

    def __str__(self):
        return self.toYamlString()

class FluentPoseModification(Fluent):

    yaml_tag='!FluentPoseModification'
    GROUP_CHOICE_MAP = {}

    def __setstate__(self,state):
        """
        PyYaml does not call __init__. this is an init replacement.
        """
        # apply group state...
        if not FluentPoseModification.GROUP_CHOICE_MAP.has_key(state['Group']):
            choice = random.randint(0, state['Choices']-1)
            FluentPoseModification.GROUP_CHOICE_MAP[state['Group']] = choice
        
        self.choice = FluentPoseModification.GROUP_CHOICE_MAP[state['Group']]

        self.Instance = state['Instance']

        self.Modifications = state['Modifications']

        self.Attachments = state['Attachments']

        # fix for bug:
        #if type(self.Class_Instance[0]) == type(False):
        #    self.Class_Instance[0] = '"On"'
        #if self.Class_Instance[0] == 'On':
        #    self.Class_Instance[0] = '"On"'
        #self.StartTime = state['StartTime']
        #self.FinishTime = state['FinishTime']

    def getMod(self, propertyString):
        for mod in self.Modifications:
            if mod[0] == propertyString:
                return mod[1][self.choice]
        return 0
        

if __name__ == '__main__':
    # filepath = roslib.packages.get_pkg_dir('race_simulation_run') + '/data/test.yaml'
    # changedFilepath = roslib.packages.get_pkg_dir('race_simulation_run') + '/data/output.yaml'
    # replacementsPath = roslib.packages.get_pkg_dir('race_simulation_run') + '/data/replace.yaml'

    # # load fluents:
    # fluents = []
    # with open(filepath) as f:
    #     for fluent in yaml.load_all(f):
    #         fluents.append(fluent)
    
    # # load modifications:
    # fluentPoseModifications = []
    # with open(replacementsPath) as f:
    #     for poseMod in yaml.load_all(f):
    #         fluentPoseModifications.append(poseMod)
            
    # # modify fluents poses:
    # for fluent in fluents[:]:
    #     for poseMod in fluentPoseModifications:
    #         if (poseMod.Instance == fluent.Class_Instance[1]) \
    #                 or (fluent.Class_Instance[1] in poseMod.Attachments):
    #             for prop in fluent.properties:
    #                 if poseMod.getMod(prop.role_type) != 0:
    #                     prop.role_filler += poseMod.getMod(prop.role_type)
                

    # # generate new file:
    # with open(changedFilepath, 'w') as cf:
    #     string = ('---\n').join(str(fluent) for fluent in fluents)
    #     cf.write(string)
    
    # load initial knowledge and spawn objects fluents
    initialPath = roslib.packages.get_pkg_dir('race_static_knowledge') + '/data/race_initial_knowledge.yaml'
    spawnPath = roslib.packages.get_pkg_dir('race_simulation_run') + '/data/spawn_objects.yaml'
    replacementsPath = roslib.packages.get_pkg_dir('race_simulation_run') + '/data/replace.yaml'
    #replacementsPath = roslib.packages.get_pkg_dir('race_simulation_run') + '/data/replace_simple.yaml'
    tempFilePath = roslib.packages.get_pkg_dir('race_simulation_run') + '/data/output.yaml'

    # load fluents:
    fluents = []
    with open(initialPath) as f:
        for fluent in yaml.load_all(f):
            fluents.append(fluent)
    with open(spawnPath) as f:
        for fluent in yaml.load_all(f):
            fluents.append(fluent)

    # load modifications:
    fluentPoseModifications = []
    with open(replacementsPath) as f:
        for poseMod in yaml.load_all(f):
            fluentPoseModifications.append(poseMod)
           
    # modify fluents poses:
    for fluent in fluents[:]:
        for poseMod in fluentPoseModifications:
            if (poseMod.Instance == fluent.Class_Instance[1]) \
                    or (fluent.Class_Instance[1] in poseMod.Attachments):
                for prop in fluent.properties:
                    if poseMod.getMod(prop.role_type) != 0:
                        prop.role_filler += poseMod.getMod(prop.role_type)

    # generate new file:
    with open(tempFilePath, 'w') as cf:
        string = ('---\n').join(str(fluent) for fluent in fluents)
        cf.write(string)
