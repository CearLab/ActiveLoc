 % check if the neighborhood mesh is rigid
        function [isRigid, e] = isRigid(losTAB,agents_list)

            % init
            isRigid = 0;

            % safety
            if isempty(losTAB)
                return;
            end
            
            % get rigidity matrix
            R = calcRigitdyMatrix(losTAB ,agents_list);

            % get nonzero eigs
            e = eig(R'*R);
        
            % get # nnz elements
            pos = find(abs(e) < 1e-10);

            e(pos) = [];
            isRigid = (numel(pos)==3);

        end