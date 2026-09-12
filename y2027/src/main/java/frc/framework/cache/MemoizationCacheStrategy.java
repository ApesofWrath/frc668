package frc.framework.cache;

import frc.framework.ExecutionManager;
import frc.framework.SystemCachableResult;

public class MemoizationCacheStrategy implements CacheStrategy {

    @Override
    public boolean isCacheValid(long time, ExecutionManager manager, SystemCachableResult previousResult) {
        return previousResult.doInputsMatchWithManager(manager);
    }
    
}
