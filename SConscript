from building import *

cwd = GetCurrentDir()
path = [cwd+'/Inc']
src  = Glob('Src/*.c')

if not GetDepend('AT_USING_SOCKET'):
    SrcRemove(src, ['Src/at_socket_combo.c'])
    
if GetDepend(['AT_DEVICE_COMBO_SAMPLE']):
    src += Glob('examples/at_sample_combo.c')

 
group = DefineGroup('combo_at', src, depend = ['AT_DEVICE_USING_COMBO'], CPPPATH = path)

Return('group')