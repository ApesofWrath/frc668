package frc.framework.cache;

import frc.framework.execution.ExecutionManager;
import frc.framework.execution.SystemCachableResult;

/**
 * A caching strategy for 'pure' systems, that produce no side effects and have the same outputs given the same inputs.
 */
public class MemoizationCacheStrategy implements CacheStrategy {
	@Override
	public boolean isCacheValid(long time, ExecutionManager manager, SystemCachableResult previousResult) {
		return previousResult.doInputsMatchWithManager(manager);
	}
}
