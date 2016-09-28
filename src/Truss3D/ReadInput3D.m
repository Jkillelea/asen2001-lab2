function [Joints_Array, MemberConnectivity_Array, ReactionJoints_Array, ReactionVector_Array, LoadJoints_Array, LoadVectors_Array] = ReadInput3D(InputFile)
% function [Joints_Array, MemberConnectivity_Array, ReactionJoints_Array, ReactionVector_Array, LoadJoints_Array, LoadVectors_Array] = readinput(InputFile)
%
% read input file
%
% input: inputfile - name of input file
%
% output: Joints_Array             - coordinates of joints
%         MemberConnectivity_Array - connectivity
%         ReactionJoints_Array     - joint id where reaction acts on
%         ReactionVector_Array     - unit vector associated with reaction force
%         LoadJoints_Array         - joint id where external load acts on
%         LoadVectors_Array        - load vector
%

% open inputfile
fid = fopen(InputFile);

if fid < 0;error('inputfile does not exist');end

% initialze counters and input block id
counter = 0;
inpblk  = 1;

% read first line
line = fgetl(fid);

% read input file
while line > 0
    % check if comment
    if strcmp(line(1),'#')
        % read next line and continue
        line = fgetl(fid);
        continue;
    end

    switch inpblk
        case 1 % read number of joints, bars, reactions, and loads

            dims=sscanf(line,'%d%d%d%d%d');

            numjoints = dims(1);
            numbars   = dims(2);
            numreact  = dims(3);
            numloads  = dims(4);

            % check for correct number of reaction forces
            % if numreact~=3; error('incorrect number of reaction forces');end

            % initialize arrays
            Joints_Array       = zeros(numjoints,3);
            MemberConnectivity_Array = zeros(numbars,2);
            ReactionJoints_Array   = zeros(numreact,1);
            ReactionVector_Array     = zeros(numreact,3);
            LoadJoints_Array   = zeros(numloads,1);
            LoadVectors_Array     = zeros(numloads,3);

            % check whether system satisfies static determiancy condition
            % if 2*numjoints - 3 ~= numbars
            % error('truss is not statically determinate');
            % end

            % expect next input block to be joint coordinates
            inpblk = 2;

        case 2 % read coordinates of joints

            % increment joint id
            counter = counter + 1;

            % read joint id and coordinates;
            tmp = sscanf(line,'%d%e%e');

            % extract and check joint id
            jointid = tmp(1);
            if jointid > numjoints || jointid < 1
                error('joint id number need to be smaller than number of joints and larger than 0');
            end

            % store coordinates of joints
            Joints_Array(jointid,:)=tmp(2:4);

            % expect next input block to be connectivity
            if counter == numjoints
                inpblk  = 3;
                counter = 0;
            end

        case 3 % read connectivity of bars

            % increment bar id
            counter = counter + 1;

            % read connectivity;
            tmp = sscanf(line,'%d%d%d');

            % extract bar id number and check
            barid = tmp(1);
            if barid > numbars || barid < 0
                error('bar id number needs to be smaller than number of bars and larger than 0');
            end

            % check joint ids
            if max(tmp(2:3)) > numjoints || min(tmp(2:3)) < 1
                error('joint id numbers need to be smaller than number of joints and larger than 0');
            end

            % store connectivity
            MemberConnectivity_Array(barid,:) = tmp(2:3);

            % expect next input block to be reaction forces
            if counter == numbars
                inpblk  = 4;
                counter = 0;
            end

        case 4 % read reation force information

            % increment reaction id
            counter = counter + 1;

            % read joint id and unit vector of reaction force;
            tmp = sscanf(line,'%d%e%e');

            % extract and check joint id
            jointid = tmp(1);
            if jointid > numjoints || jointid < 1
                error('joint id number need to be smaller than number of joints and larger than 0');
            end

            % extract untit vector and check length
            uvec = tmp(2:4);
            uvec = uvec/norm(uvec);

            % store joint id and unit vector
            ReactionJoints_Array(counter)  = jointid;
            ReactionVector_Array(counter,:)  = uvec;

            % expect next input block to be external loads
            if counter == numreact
                inpblk  = 5;
                counter = 0;
            end

        case 5 % read external load information

            % increment reaction id
            counter = counter + 1;

            % read joint id and unit vector of reaction force;
            tmp = sscanf(line,'%d%e%e');

            % extract and check joint id
            jointid = tmp(1);
            if jointid > numjoints || jointid < 1
                error('joint id number need to be smaller than number of joints and larger than 0');
            end

            % extract force vector
            frcvec = tmp(2:4);

            % store joint id and unit vector
            LoadJoints_Array(counter)    = jointid;
            LoadVectors_Array(counter,:) = frcvec;

            % expect no additional input block
            if counter == numloads
                inpblk  = 99;
                counter = 0;
            end

        otherwise
            %fprintf('warning: unknown input: %s\n',line);
    end

    % read next line
    line = fgetl(fid);
end

% close input file
fclose(fid);

end
