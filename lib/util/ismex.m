function flag = ismex(name)
%ISMEX Checks if there exists a MEX function with the given name.
    flag = (exist(name, 'file') == 3);
end
