import numpy as np



def read_automeasure_toneslist(filename,printing=True):
    """
    Read a standard tones list file from the cardiff kids lab automeasure program.
    """
    try:
        result =np.genfromtxt(filename,names=True,delimiter='\t',dtype=('S32','f8','f8','i4','i4'),unpack=True)
        name,freqs,offset_att,all_flag,none_flag = result
        if printing:
            print('Read %d tones from %s'%(len(freqs),filename))
            print('Name\tFreq\tOffset att\tAll\tNone')
            for k in range(len(freqs)):
                print('%s\t%f\t%.2f\t%d\t%d'%(name[k].decode(),freqs[k],offset_att[k],all_flag[k],none_flag[k]))

        return name,freqs,offset_att,all_flag,none_flag

    except FileNotFoundError:
        print(f'File not found {filename}')
    except Exception as e:
        print('Unrecognised file format, is this a standard lab ToneList file?')
        raise e



def write_automeasure_toneslist(filename,freqs,names=None,offset_att=None,all_flag=None,none_flag_flag=None,amps=None):
    """
    Write a standard tones list file for the cardiff kids lab automeasure program.

    If <amps> is given, offset_att is calculated from the amplitudes, overwriting any value given in <offset_att>.
    """
    if names is None:
        names=['K%03d'%k for k in range(len(freqs))]
    if offset_att is None:
        offset_att = [0]*len(freqs)
    if all_flag is None:
        all_flag = [1]*len(freqs)
    if none_flag_flag is None:
        none_flag_flag = [0]*len(freqs)
    if amps is not None:
        att = 20*np.log10(np.abs(amps))
        att-= att.min()
        #att = 2*floor(att/2.)
        #att[att>62]=62
        offset_att=att

    file=open(filename,'w')
    file.write('Name\tFreq\tOffset att\tAll\tNone\r\n')
    for k in range(len(freqs)):
        file.write('%s\t%f\t%f\t%d\t%d\r\n'%(names[k],freqs[k],offset_att[k],all_flag[k],none_flag_flag[k]))
    file.close()
    return

