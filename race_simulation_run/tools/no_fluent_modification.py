#! /usr/bin/env python
import yaml
import random
import roslib

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
    tempFilePath = roslib.packages.get_pkg_dir('race_simulation_run') + '/data/output.yaml'

    # load fluents:
    fluents = []
    with open(initialPath) as f:
        for fluent in yaml.load_all(f):
            fluents.append(fluent)
    with open(spawnPath) as f:
        for fluent in yaml.load_all(f):
            fluents.append(fluent)

    # generate new file:
    with open(tempFilePath, 'w') as cf:
        string = ('---\n').join(str(fluent) for fluent in fluents)
        cf.write(string)
