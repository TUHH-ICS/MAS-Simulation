function mat = makeHandle(mat)
%MAKEHANDLE Wraps a given matrix into a function handle with one argument.
%   This function checks if its argument is a function handle and if not,
%   wraps the matrix into a function handle with one argument. This ensures
%   that the returned value is a function handle if it wasn't one before.

if ~isa(mat, 'function_handle')
    mat = @(~) mat;
end
end

