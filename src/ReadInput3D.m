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

    if fid < 0
        error('inputfile does not exist');
    end

    % read number of joints, bars, reactions, loads
    tmp = sscanf(next_non_comment_line(fid),'%d%d%d%d%d');

    numjoints = tmp(1);
    numbars   = tmp(2);
    numreact  = tmp(3);
    numloads  = tmp(4);

    % check for correct number of reaction forces
    if numreact~=6
        error('Incorrect number of reaction forces. Need 6 for a 3D object.');
    end

    % initialize arrays
    Joints_Array             = zeros(numjoints, 3);
    MemberConnectivity_Array = zeros(numbars,   2);
    ReactionJoints_Array     = zeros(numreact,  1);
    ReactionVector_Array     = zeros(numreact,  3);
    LoadJoints_Array         = zeros(numloads,  1);
    LoadVectors_Array        = zeros(numloads,  3);

    % check whether system satisfies static determiancy condition
    if 3*numjoints - 6 ~= numbars
        error('Truss is not statically determinate. NumJoints %d, NumBars %d. Should be %d bars.', ...
         numjoints, numbars, (3*numjoints - 6));
    end


    % read coordinates of joints
    for i = 1:numjoints
      tmp = sscanf(next_non_comment_line(fid),'%d%e%e%e');

      % extract and check joint id
      jointid = tmp(1);
      if ((jointid > numjoints) || (jointid < 1))
        error('Joint id number need to be smaller than number of joints and larger than 0');
      end
      % store coordinates of joints
      Joints_Array(jointid,:) = tmp(2:4);
    end


    % Read connectivity
    for i = 1:numbars
        tmp = sscanf(next_non_comment_line(fid),'%d%d%d');

        % extract bar id number and check
        barid = tmp(1);
        if ((barid > numbars) || (barid < 0))
            error('Bar id number needs to be smaller than number of bars and larger than 0');
        end

      % check joint ids
      if (max(tmp(2:3)) > numjoints) || (min(tmp(2:3)) < 1)
          error('Joint id numbers need to be smaller than number of joints and larger than 0');
      end

      % store connectivity
      MemberConnectivity_Array(barid,:) = tmp(2:3);
    end


    % Read reaction force information
    for i = 1:numreact
        tmp = sscanf(next_non_comment_line(fid),'%d%e%e');

        % extract and check joint id
        jointid = tmp(1);
        if (jointid > numjoints) || (jointid < 1)
            error('Joint id number need to be smaller than number of joints and larger than 0');
        end

        % extract untit vector and check length
        uvec = tmp(2:4);
        uvec = uvec/norm(uvec);

        % store joint id and unit vector
        ReactionJoints_Array(i)   = jointid;
        ReactionVector_Array(i,:) = uvec;
    end

    % Read external load information
    for i = 1:numloads
        tmp = sscanf(next_non_comment_line(fid),'%d%e%e');
        % extract and check joint id
        jointid = tmp(1);
        if jointid > numjoints || jointid < 1
            error('Joint id number need to be smaller than number of joints and larger than 0');
        end

        % extract force vector
        frcvec = tmp(2:4);

        % store joint id and unit vector
        LoadJoints_Array(counter)    = jointid;
        LoadVectors_Array(counter,:) = frcvec;
    end

    % close input file
    fclose(fid);
end


% helper function, only available from the context of the function above.
function l = next_non_comment_line(fileID)
  % Returns next line that DOESN'T begin with a '#'
  l = fgets(fileID);
  while l(1) == '#'
    l = fgets(fileID);
  end
end
