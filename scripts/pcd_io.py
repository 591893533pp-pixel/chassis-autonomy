from pathlib import Path
import numpy as np

def read_pcd(path):
    with Path(path).open('rb') as f:
        header={}
        while True:
            line=f.readline()
            if not line:raise ValueError('Missing PCD DATA header')
            words=line.decode('ascii').split()
            if not words or words[0].startswith('#'):continue
            header[words[0]]=words[1:]
            if words[0]=='DATA':break
        n=int(header['POINTS'][0]);names=header['FIELDS'];counts=list(map(int,header.get('COUNT',['1']*len(names))))
        fields=[]
        for name,t,size,count in zip(names,header['TYPE'],header['SIZE'],counts):
            kind={'F':'f','I':'i','U':'u'}[t];dt='<'+kind+size
            fields.append((name,dt) if count==1 else (name,dt,(count,)))
        if header['DATA']==['binary']:
            dtype=np.dtype(fields);data=np.fromfile(f,dtype=dtype,count=n)
            if len(data)!=n:raise ValueError('Truncated PCD')
        elif header['DATA']==['ascii']:
            raw=np.loadtxt(f,ndmin=2);data=np.empty(n,dtype=fields)
            if raw.shape!=(n,len(fields)) or any(c!=1 for c in counts):raise ValueError('Unsupported ASCII PCD')
            for i,k in enumerate(names):data[k]=raw[:,i]
        else:raise ValueError('Expected uncompressed binary or ASCII PCD')
    xyz=np.column_stack([data[k] for k in ('x','y','z')]);return xyz[np.isfinite(xyz).all(axis=1)]
