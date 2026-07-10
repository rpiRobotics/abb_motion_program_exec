from importlib import resources as impre
import xml.etree.ElementTree as ET
import shutil
import os
import re

def project_setup(args):

    # Set working directory
    wdir = os.getcwd()
    if args.inplace:
        print("Attempting to configure current directory...")
    else:
        if args.dir:
            wdir=args.dir
        else:
            wdir = os.path.expanduser("~") + "\\Documents\\RobotStudio\\Projects\\"
        wdir = wdir + args.project
        print("Attempting to configure project : {}\\".format(wdir))
        os.chdir(wdir)
    if args.egm:
        print("    Option: Externally Guided Motion")


    # Set up controller(s)
    try:
        os.chdir("Controller Data")
        with os.scandir() as ctrlrs:
            for cntrlr in ctrlrs:
                if not cntrlr.is_file(): # is a controller directory
                    apply_setup(cntrlr.name,egm=args.egm)
    except FileNotFoundError:
        print("Warning: No Controllers found")
        return
    # Delete any existing virtual controllers (RobotStudio will automatically
    # regenerate with modified configurations
    os.chdir('..')
    shutil.rmtree("Virtual Controllers")


def apply_setup(cntrlr,egm=False):
    # Lists of files to modify
    hfiles = ['error_reporter.mod',
             'motion_program_exec.mod',
             'motion_program_logger.mod',
             'motion_program_shared.sys']
    cfiles = ['SYS.cfg','EIO.cfg']
    # Options to add to system.xml and backinfo.txt
    options = ['616-1 PC Interface',
               '623-1 Multitasking']
    options_short = ['pcinterface','multitasking']
    # RAPID tasks tree
    rapidtasks = {'TASK0':[[],['motion_program_shared.sys']],
                  'TASK1':[['motion_program_exec.mod'],['user.sys']],
                  'TASK2':[['motion_program_logger.mod'],['user.sys']],
                  'TASK3':[['error_reporter.mod'],['user.sys']]}
    # backinfo.txt tail append file
    backinfotail = 'backinfo.tail'

    from .robot import HOME as rhome
    from .robot import autosetup as rautosetup
    if egm:
        from .robot import config_params_egm as config_params
        hfiles.append('motion_program_exec_egm.mod')
        cfiles.append('MOC.cfg')
        options.append('689-1 Externally Guided Motion (EGM)')
        options.append('UDPUC Driver')
        options_short.append('externallyguidedmotion')
        options_short.append('udpuc')
        backinfotail = 'backinfo_egm.tail'
        rapidtasks['TASK1'] = [['motion_program_exec_egm.mod','motion_program_exec.mod'],['user.sys']]
    else:
        from .robot import config_params as config_params

    print('Configuring controller {}'.format(cntrlr))

    ### Modifying files

    # Copying HOME files
    for file in hfiles:
        src = impre.files(rhome).joinpath(file)
        shutil.copyfile(src,"{}\\HOME\\{}".format(cntrlr,file))

    # Setting config options in system.xml
    cfgfile = ET.parse('{}\\system.xml'.format(cntrlr))
    cfgroot = cfgfile.getroot()
    cfgmod = cfgroot.find('ControlModule')
    for option in options:
        ET.SubElement(cfgmod,'Option',descr=option)
    ET.indent(cfgfile)
    cfgfile.write('{}\\system.xml'.format(cntrlr),short_empty_elements=False,encoding="utf-8",xml_declaration=True)

    # Setting config options in BACKINFO\controller.rsf
    cfgfile = ET.parse('{}\\BACKINFO\\controller.rsf'.format(cntrlr))
    cfgroot = cfgfile.getroot()
    cfgstg = cfgroot.find('.//Settings')
    for option,soption in zip(options,options_short):
        ET.SubElement(cfgstg,'Setting',id='abb.robotics.robotware.options.{}'.format(soption),displayName=option,robot='1')
    ET.indent(cfgfile)
    cfgfile.write('{}\\BACKINFO\\controller.rsf'.format(cntrlr),encoding="utf-8",xml_declaration=True)
    

    # SYSPAR files
    for file in cfiles:
        src = impre.files(config_params).joinpath(file)
        dst = '{}\\SYSPAR\\{}'.format(cntrlr,file)
        merge_cfg(src,dst)

    # BACKINFO/backinfo.txt
    fbi = open("{}\\BACKINFO\\backinfo.txt".format(cntrlr),'r',newline='\r\n')
    fbid = fbi.read()
    fbi.close()
    # Trim end of file where tasks start
    fbid = fbid.split(">>TASK1")[0]
    # Append the required tasks
    bitailfile = impre.files(rautosetup).joinpath(backinfotail)
    with open(bitailfile,'r',newline='\r\n') as bif:
        fbid += bif.read()
    # add the options
    # first convert to lines
    fbid = fbid.split('\r\n')
    for idx,option in enumerate(options):
        fbid.insert(8+idx,"  "+option) # add spaces to match format of backinfo.txt
    # Overwrite original file
    with open("{}\\BACKINFO\\backinfo.txt".format(cntrlr),'w',newline='\r\n') as bio:
        for line in fbid:
            bio.write(line+'\n')

    # Building RAPID tasks
    shutil.rmtree("{}\\RAPID".format(cntrlr))
    for task in rapidtasks.keys():
        progdir = '{}\\RAPID\\{}\\PROGMOD'.format(cntrlr,task)
        sysdir = '{}\\RAPID\\{}\\SYSMOD'.format(cntrlr,task)
        os.makedirs(progdir)
        os.makedirs(sysdir)
        for file in rapidtasks[task][0]:
            shutil.copy('{}\\HOME\\{}'.format(cntrlr,file),progdir)
        for file in rapidtasks[task][1]:
            shutil.copy('{}\\HOME\\{}'.format(cntrlr,file),sysdir)

    print("Done!")

def merge_cfg(src,dst):
    src_cfg = read_cfg(src)
    dst_cfg = read_cfg(dst,removetask=True)
    for skey,sval in src_cfg.items():
        if skey == 'HEADER': # skip the header
            continue
        if not skey in dst_cfg:
            dst_cfg[skey] = ''
        dst_cfg[skey] += sval
    write_cfg(dst,dst_cfg)

def read_cfg(src,removetask=False):
    cfg_dict = {}
    with open(src,'r') as fcfg:
        line = fcfg.readline()
        key = 'HEADER'
        while line:
            if not key:
                key = fcfg.readline()
                line = fcfg.readline()
            val = ''
            while line:
                if line[0] == '#':
                    break
                if removetask:
                    if key.startswith("CAB_TASKS"):
                        if "MotionTask" in line:
                            val = val[:-1]
                            line = fcfg.readline()
                            continue
                val += line
                line = fcfg.readline()
            cfg_dict[key] = val
            key = ''
    return cfg_dict

def write_cfg(dst,cfg):
    with open(dst,'w') as fdst:
        fdst.write(cfg['HEADER'])
        for ckey,cval in cfg.items():
            if ckey == 'HEADER':
                continue
            fdst.write('#\n')
            fdst.write(ckey)
            fdst.write(cval)
