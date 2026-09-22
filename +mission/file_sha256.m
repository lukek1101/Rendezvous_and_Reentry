function hex=file_sha256(path)
fid=fopen(path,'rb');
if fid<0, error('mission:FileRead','Cannot read %s',path); end
cleanup=onCleanup(@()fclose(fid));
bytes=fread(fid,Inf,'*uint8');
digest=java.security.MessageDigest.getInstance('SHA-256');
digest.update(bytes);
hex=lower(string(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[])));
end
