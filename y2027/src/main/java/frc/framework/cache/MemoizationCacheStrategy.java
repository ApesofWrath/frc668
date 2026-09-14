package frc.framework.cache;

import frc.framework.execution.ExecutionManager;
import frc.framework.execution.SystemCachableResult;

public class MemoizationCacheStrategy implements CacheStrategy {
	@Override
	public boolean isCacheValid(long time, ExecutionManager manager, SystemCachableResult previousResult) {
		return previousResult.doInputsMatchWithManager(manager);
	}
}
