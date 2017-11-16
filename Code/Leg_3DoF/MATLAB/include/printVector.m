function printVector(v, name)
    if (size(v,1) ~= 1 && size(v,2) ~= 1)
        error('vector is not 1D!');
    end
    
    fprintf('%s = [\n', name);
    for i=1:length(v)
        fprintf('%16.4f\n', v(i));
    end
    fprintf(']\n');
end