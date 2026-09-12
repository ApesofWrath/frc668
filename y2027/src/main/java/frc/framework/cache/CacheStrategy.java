package frc.framework.cache;

import frc.framework.ExecutionManager;
import frc.framework.SystemCachableResult;

public interface CacheStrategy {
    public boolean isCacheValid(long time, ExecutionManager manager, SystemCachableResult previousResult);
}
