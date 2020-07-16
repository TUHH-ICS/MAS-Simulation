% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef(Sealed) ProbingValue
    %PROBINGVALUE Utility class that allows to probe the size of values
    %that are return by function handles.
    %   In general nonlinear and LPV systems the properties are non
    %   determined by matrices as in LTI systems, but be vector- or
    %   matrix-valued functions, respectively. Their size cannot easily be
    %   queried using the size() function. To calculate their size, we must
    %   pass in vectors as function arguments, which size is however also
    %   not known.
    %
    %   This class solves this problem by implementing a way to represent
    %   vectors and matrices with at least some dimensions unknown. The
    %   usual arithmetic can be performed on these values, where the
    %   dimensions get set as they become known along the way.
    
    properties(GetAccess = private, SetAccess = immutable)
        % Contains the dimension of the value. Dimension of unknown length
        % are represented as NaN values in this array.
        dim
    end

    methods(Access = private)
        function obj = ProbingValue(dims)
            %PROBINGVALUE Create an instance of this class
            %   Trailing dimension with length 1 gets stripped, matching
            %   the behaviour of Matlab for standard arrays.
            %
            %   This constructor cannot be called directly, instead create
            %   values using the static withDimension() function, i. e.
            %   call val = ProbingValue.withDimension(...);
            
            obj.dim = dims;
            
            % Strip trailing 1s in the dimensions
            while length(obj.dim) > 2 && obj.dim(end) == 1
                obj.dim(end) = [];
            end
        end
    end
    
    methods(Static, Access = public)
        function obj = withDimension(varargin)
            %WITHDIMENSION Creates a ProbingValue of the specified
            %dimensions.
            %   This function checks if any of the given dimensions is
            %   unkown. If not, then it will return a double array with the
            %   given dimension, otherwise it returns a matching
            %   ProbingValue. Unknown dimensions can be declared using
            %   either [], i. e. an empty array, or NaN.
            
            if any(~cellfun(@isnumeric, varargin))
                error('Size inputs must be numeric.')
            end
            
            % Replace empty vectors with 1x1 NaN vectors
            mask = cellfun(@isempty, varargin);
            varargin(mask) = {NaN};
            
            if nargin == 0
                dims = 1;
            elseif nargin == 1
                if isscalar(varargin{1})
                    dims = [varargin{1}, 1];
                else
                    dims = varargin{1};
                end
            else
                if any(~cellfun(@isscalar, varargin))
                    error('Size inputs must be scalar.')
                end
                
                dims = [varargin{:}];
            end
            
            % Check if any dimension is unknown and create the return value
            % accordingly.
            if any(isnan(dims))
                obj = ProbingValue(dims);
            else
                obj = zeros(dims);
            end
        end
    end
    
    methods(Access = public)       
        function [sz, varargout] = size(A, varargin)           
            if nargin >= 2
                if nargin == 2
                    dims = varargin{1};
                elseif any(~cellfun(@isscalar, varargin))
                    error('Dimension argument must be a positive integer scalar within indexing range.')
                else
                    dims = [varargin{:}];
                end
                
                mask      = dims <= length(A.dim);
                sz(mask)  = A.dim(dims(mask));
                sz(~mask) = 1;
            else
                sz = A.dim;
            end

            if nargout >= 2 
                if nargin >= 2
                    if length(dims) == nargout
                        varargout = num2cell(sz(2:end));
                    else
                        error('Incorrect number of output arguments. Number of output arguments must equal the number of input dimension arguments.')
                    end
                else
                    val = min(nargout-1, length(sz));
                    varargout(1:val-1)       = num2cell(sz(2:val));
                    varargout(val:nargout-2) = {1};
                    varargout{nargout-1}     = prod(sz(val+1:end));
                end
                
                sz = sz(1);
            end
        end
        
        function n = numel(A)
            n = prod(size(A)); %#ok<PSIZE>
        end
        
        function sref = subsref(A, s)
            switch s(1).type
                case '.'
                    sref = builtin('subsref', A, s);
                case '()'
                    dims = zeros(size(s.subs));
                    for i = 1:length(s.subs)
                        if i > length(A.dim)
                            nElem = 1;
                        elseif i == length(s.subs)
                            nElem = prod(A.dim(i:end));
                        else
                            nElem = A.dim(i);
                        end

                        ind = s.subs{i};
                        if ind == ':'
                            dims(i) = nElem;
                        elseif islogical(ind)
                            if isnan(nElem) || find(ind, 1, 'last') < nElem
                                dims(i) = sum(ind);
                            else
                                error('The logical indices contain a true value outside of the array bounds')
                            end
                        elseif isnumeric(ind)
                            if all(isfinite(ind)) && all(mod(ind, 1) == 0) && min(ind) >= 1
                                if isnan(nElem) || max(ind) <= nElem
                                    dims(i) = length(ind);
                                else
                                    error('Index exceeds the number of array elements (%d)', nElem)
                                end
                            elseif length(ind) == 1 && isnan(ind) && isnan(nElem)
                                dims(i) = NaN;                                    
                            else
                                error('Array indices must be positive integers or logical values.')
                            end
                        else
                            error('Array indices must be positive integers or logical values.')
                        end
                    end
                    
                    if isempty(dims)
                        dims = A.dim;
                    end
                    
                    sref = ProbingValue.withDimension(dims);
                case '{}'
                    sref = builtin('subsref', A, s);
            end
        end
        
        function ind = end(A,k,n)
           sz = size(A);
           if k < n
              ind = sz(k);
           else
              ind = prod(sz(k:end));
           end
        end
        
        function C = horzcat(A, B, varargin)
            C = directedcat(2, A, B, varargin{:});
        end
        
        function C = vertcat(A, B, varargin)
            C = directedcat(1, A, B, varargin{:});
        end
        
        function B = squeeze(A)
            B = ProbingValue.withDimension(A.dim(A.dim ~= 1));
        end
        
        function C = plus(A, B)
            dims = checkCompatibility(A, B);
            if isempty(dims)
                error('Array dimensions must match for binary array op.')
            else
                C = ProbingValue.withDimension(dims);
            end
        end
        
        function C = minus(A, B)
            C = plus(A, -B);
        end
        
        function B = uplus(A)
            B = A;
        end
        
        function B = uminus(A)
            B = A;
        end
        
        function C = times(A, B)
            dims = checkCompatibility(A, B);
            if isempty(dims)
                error('Array dimensions must match for binary array op.')
            else
                C = ProbingValue.withDimension(dims);
            end
        end
        
        function C = mtimes(A, B)
            if isscalar(A)
                C = ProbingValue.withDimension(size(B));
            elseif isscalar(B)
                C = ProbingValue.withDimension(size(A));
            else
                szA = size(A);
                szB = size(B);
                
                if any(szA(3:end) ~= 1) || any(szB(3:end))
                    error('Arguments must be 2-D, or at least one argument must be scalar.')
                elseif isnan(szA(2)) || isnan(szB(1)) || szA(2) == szB(1)
                    C = ProbingValue.withDimension(szA(1), szB(2));
                else
                    error('Incorrect dimensions for matrix multiplication. Check that the number of columns in the first matrix matches the number of rows in the second matrix.')
                end
            end
        end
        
        function C = rdivide(A, B)
            dims = checkCompatibility(A, B);
            if isempty(dims)
                error('Array dimensions must match for binary array op.')
            else
                C = ProbingValue.withDimension(dims);
            end
        end
        
        function C = ldivide(B, A)
            dims = checkCompatibility(B, A);
            if isempty(dims)
                error('Array dimensions must match for binary array op.')
            else
                C = ProbingValue.withDimension(dims);
            end
        end
        
        function C = power(A, B)
            dims = checkCompatibility(A, B);
            if isempty(dims)
                error('Array dimensions must match for binary array op.')
            else
                C = ProbingValue.withDimension(dims);
            end
        end
        
        function C = mpower(A, b)
            if length(A.dim) > 2 && any(A.dim(3:end) ~= 1)
                error('Arguments must be 2-D.')
            elseif isscalar(b) && A.dim(1) == A.dim(2)
                C = A;
            else
                error ('Incorrect dimensions for raising a matrix to a power. Check that the matrix is square and the power is a scalar.')
            end
        end
        
        function B = transpose(A)
            if length(A.dim) > 2 && any(A.dim(3:end) ~= 1)
                error('Transpose on ND array is not defined. Use PERMUTE instead.')
            else
                B = ProbingValue.withDimension(A.dim([2, 1]));
            end
        end
        
        function B = ctranspose(A)
            B = transpose(A);
        end
        
        function B = permute(A, dimorder)
            if ~isnumeric(dimorder)
                error('ORDER must be a numeric array.')
            elseif length(dimorder) < length(A.dim)
                error('ORDER must have at least N elements for an N-D array.')
            else
                dims = ones(1, length(dimorder));
                flag = zeros(1, length(dimorder), 'logical');
                
                for i = 1:length(dims)
                    if dimorder(i) <= 0 || dimorder(i) > length(dimorder)
                        error('ORDER contains an invalid permutation index.')
                    elseif flag(dimorder(i))
                        error('ORDER cannot contain repeated permutation indices.')
                    else
                        flag(dimorder(i)) = true;
                        if dimorder(i) <= length(A.dim)
                            dims(i) = A.dim(dimorder(i));
                        end
                    end
                end
                
                B = ProbingValue.withDimension(dims);
            end
        end
        
        function Y = sqrt(X)
            Y = X;
        end
        
        function Y = sin(X)
            Y = X;
        end
        
        function Y = cos(X)
            Y = X;
        end
        
        function Y = tan(X)
            Y = X;
        end
        
        function Y = exp(X)
            Y = X;
        end
        
        function Y = log(X)
            Y = X;
        end
        
        function Y = log10(X)
            Y = X;
        end
        
        function Y = factorial(X)
            Y = X;
        end
    end
end

function dims = checkCompatibility(A, B)
    %CHECKCOMPATABILITY Checks if two values have compatible dimensions as
    %specified by the Matlab documentation.
    %   This function extends the compatibility check to ProbingValues,
    %   i. e. values of unknown size. The unkown dimensions are considered
    %   compatible. Thus this function not only checks the dimensions for
    %   compatibility, but also calculates the resulting dimensions. If the
    %   dimensions are not matching, an empty vector is returned.
    
    szA = size(A);
    szB = size(B);
    
    % The resulting array will have the same dimension as the higher
    % dimensional source array.
    dims = zeros(1, max(length(szA), length(szB)));
    
    % Only check up to the dimension of the lower dimensional array, as all
    % trailing dimension are considered matching by Matlab.
    checked = min(length(szA), length(szB));
    for i = 1:checked
        if isnan(szA(i)) || szA(i) == 1     % Dimension of A unknown or compatible
            dims(i) = szB(i);
        elseif isnan(szB(i)) || szB(i) == 1 % Dimension of B unknown or compatible
            dims(i) = szA(i);
        elseif szA(i) == szB(i)             % Dimension are matching
            dims(i) = szA(i);
        else                                % Arrays not compatible
            dims = [];
            break;
        end
    end
    
    % Append all dimension that where not checked yet. This is valid as
    % Matlab would in this case extend the smaller array in the trailing
    % dimension to match the larger array.
    if ~isempty(dims)
        if length(szA) > checked
            dims(checked+1:end) = szA(checked+1:end);
        elseif length(szB) > checked
            dims(checked+1:end) = szB(checked+1:end);
        end
    end
end

function C = directedcat(dir, A, B, varargin)
    %DIRECTEDCAT Concatenates arrays of ProbingValue and double values in
    %the specified direction.
    %   This function is called by the horzcat and vertcat function to
    %   concatenate arrays which contain ProbingValue objects. The dir
    %   argument specifies in what dimension the arrays should be
    %   concatenated. 1 -> vertcat, 2 -> horzcat
    
    if ~isnumeric(dir) || ~isscalar(dir) || ~any(dir == [1, 2])
        error('The direction must be either 1 or 2.')
    end
    
    mats = [{A, B}, varargin];
    sz   = cellfun(@size, mats, 'UniformOutput', false);
    len  = cellfun(@length, sz);
    dims = zeros(1, min(len));

    % Check all dimensions, except for the one specified by dir (which must
    % be in {1, 2}.
    for i = [3-dir, 3:length(dims)]
        ind = find(cellfun(@(c) ~isnan(c(i)), sz));
        if isempty(ind)
            % If this dimension is known for no array, keep it as NaN
            dims(i) = NaN;
        else
            val = sz{ind(1)}(i);
            
            % All known lengths in this dimension must be equal
            if any(cellfun(@(c) c(i) ~= val, sz(ind(2:end))))
                error('Dimensions of arrays being concatenated are not consistent.');
            else
                dims(i) = val;
            end
        end
    end

    % In the specified dimension, all lengths are added up
    dims(dir) = sum(cellfun(@(c) c(dir), sz));
    C = ProbingValue.withDimension(dims);
end